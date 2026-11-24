// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
// Copyright (c) 2019 Intel Corporation
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "Components/BoxComponent.h"
#include "Engine/CollisionProfile.h"
#include "MovementComponents/DefaultMovementComponent.h"
#include "Rendering/SkeletalMeshRenderData.h"
#include "UObject/UObjectGlobals.h"
#include "UObject/UnrealType.h"
#include "TimerManager.h"
#include "DrawDebugHelpers.h"
#include "Kismet/KismetSystemLibrary.h"

#include "PhysXPublic.h"
#include "PhysXVehicleManager.h"
#if CARLA_CHECKPOINT_PHYSX_WHEEL_INTERNALS
#include "PxVehicleSuspWheelTire4.h"
static_assert(PX_PHYSICS_VERSION_MAJOR == 3 && PX_PHYSICS_VERSION_MINOR == 4,
              "Checkpoint wheel state requires an audit for this PhysX version");
#endif
#include "TireConfig.h"
#include "VehicleWheel.h"

#include "Carla.h"
#include "Carla/Game/CarlaHUD.h"
#include "Carla/Game/CarlaEngine.h"
#include "Carla/Game/CarlaStatics.h"
#include "Carla/Settings/EpisodeSettings.h"
#include "Carla/Trigger/FrictionTrigger.h"
#include "Carla/Util/ActorAttacher.h"
#include "Carla/Util/EmptyActor.h"
#include "Carla/Util/BoundingBoxCalculator.h"
#include "Carla/Vegetation/VegetationManager.h"
#include "Carla/Vehicle/CarlaWheeledVehicle.h"

#include <compiler/disable-ue4-macros.h>
#include <carla/rpc/VehiclePhysicsControl.h>
#include <carla/rpc/PortableVehicleCheckpoint.h>
#include <carla/rpc/VehicleLightState.h>
#include <compiler/enable-ue4-macros.h>

// This is deliberately a same-actor, same-episode prototype. In particular,
// wheel raycast caches and the PhysX scene's solver/contact state are not saved.
struct FCheckpointWheelState
{
  float Speed;
  float CorrectedSpeed;
  float Angle;
  float LowForwardSpeedTimer;
  float LowSideSpeedTimer;
  float Jounce;
};

struct FVehiclePhysicsCheckpoint
{
  FTransform Transform;
  FVector LinearVelocity;
  FVector AngularVelocity;
  FVector ObserverVelocity;
  physx::PxTransform BodyPose;
  physx::PxVec3 BodyLinearVelocity;
  physx::PxVec3 BodyAngularVelocity;
  bool bAwake;
  float WakeCounter;
  FVehicleControl InputControl;
  FVehicleControl LastControl;
  FVehicleLightState LightState;
  EVehicleInputPriority InputPriority;
  int32 RolloverTracker;
  carla::rpc::VehicleFailureState FailureState;
  carla::rpc::VehiclePhysicsControl PhysicsConfiguration;
  bool bRolloverTimerExists;
  bool bRolloverTimerPaused;
  float RolloverTimerRemaining;
  float RolloverTimerRate;
  uint32 SetupTag;
  double FixedDeltaSeconds;
  double MaxSubstepDeltaTime;
  int MaxSubsteps;
  bool bSubstepping;
  uint64 CapturedFrame;
  uint32 PhysXSceneFlags;
  const physx::PxVehicleDrive *DriveIdentity;
  physx::PxVehicleDriveDynData Drive;
  TArray<FCheckpointWheelState> Wheels;
  TArray<float> MovementFloats;
  TArray<bool> MovementBools;
};

namespace
{
carla::rpc::VehiclePhysicsControl CheckpointPhysicsConfiguration(const ACarlaWheeledVehicle &Vehicle)
{
  carla::rpc::VehiclePhysicsControl Config(Vehicle.GetVehiclePhysicsControl());
  // GetVehiclePhysicsControl reports the wheel's current position, which varies
  // with suspension motion and is not a stable configuration value.
  for (auto &Wheel : Config.wheels) Wheel.position = carla::geom::Vector3D{};
  return Config;
}

// These UPROPERTY fields are protected by the engine movement component.
// Preserving both raw and interpolated values matters for the next input tick.
const TCHAR *CheckpointFloatFields[] = {
  TEXT("RawSteeringInput"), TEXT("RawThrottleInput"), TEXT("RawBrakeInput"),
  TEXT("SteeringInput"), TEXT("ThrottleInput"), TEXT("BrakeInput"),
  TEXT("HandbrakeInput")};
const TCHAR *CheckpointBoolFields[] = {
  TEXT("bRawHandbrakeInput"), TEXT("bRawGearUpInput"), TEXT("bRawGearDownInput")};

bool CheckpointPropertiesExist(UWheeledVehicleMovementComponent *Movement)
{
  for (const TCHAR *Name : CheckpointFloatFields)
    if (!FindFProperty<FFloatProperty>(Movement->GetClass(), Name)) return false;
  for (const TCHAR *Name : CheckpointBoolFields)
    if (!FindFProperty<FBoolProperty>(Movement->GetClass(), Name)) return false;
  return true;
}
}

// =============================================================================
// -- Constructor and destructor -----------------------------------------------
// =============================================================================

ACarlaWheeledVehicle::ACarlaWheeledVehicle(const FObjectInitializer& ObjectInitializer) :
  Super(ObjectInitializer)
{
  VehicleBounds = CreateDefaultSubobject<UBoxComponent>(TEXT("VehicleBounds"));
  VehicleBounds->SetupAttachment(RootComponent);
  VehicleBounds->SetHiddenInGame(true);
  VehicleBounds->SetCollisionProfileName(UCollisionProfile::NoCollision_ProfileName);

  VelocityControl = CreateDefaultSubobject<UVehicleVelocityControl>(TEXT("VelocityControl"));
  VelocityControl->Deactivate();

  GetVehicleMovementComponent()->bReverseAsBrake = false;
  BaseMovementComponent = CreateDefaultSubobject<UBaseCarlaMovementComponent>(TEXT("BaseMovementComponent"));
}

ACarlaWheeledVehicle::~ACarlaWheeledVehicle() {}

bool ACarlaWheeledVehicle::SavePhysicsCheckpoint(const FEpisodeSettings &Settings, const FVector &ObserverVelocity)
{
#if !CARLA_CHECKPOINT_PHYSX_WHEEL_INTERNALS
  return false;
#else
  auto *Movement = GetVehicleMovementComponent();
  auto *Root = Cast<UPrimitiveComponent>(GetRootComponent());
  if (!Settings.bSynchronousMode || !Settings.FixedDeltaSeconds.IsSet() ||
      !Movement || !Movement->UpdatedComponent || !Root || !Root->IsSimulatingPhysics() ||
      !Movement->PVehicleDrive || !Movement->PVehicle ||
      !Cast<UDefaultMovementComponent>(BaseMovementComponent) ||
      bAckermannControlActive || VelocityControl->IsActive() ||
      Movement->bUseRVOAvoidance || !CheckpointPropertiesExist(Movement))
    return false;
  for (const auto &Door : ConstraintDoor)
    if (Door.Value && Door.Value->IsSimulatingPhysics()) return false;

  auto State = MakeShared<FVehiclePhysicsCheckpoint>();
  State->Transform = GetActorTransform();
  State->LinearVelocity = Root->GetPhysicsLinearVelocity();
  State->AngularVelocity = Root->GetPhysicsAngularVelocityInRadians();
  State->ObserverVelocity = ObserverVelocity;
  State->InputControl = InputControl.Control;
  State->InputPriority = InputControl.Priority;
  State->LastControl = LastAppliedControl;
  State->LightState = GetVehicleLightState();
  State->RolloverTracker = RolloverBehaviorTracker;
  State->FailureState = FailureState;
  State->PhysicsConfiguration = CheckpointPhysicsConfiguration(*this);
  if (!GetWorld()) return false;
  const auto &Timers = GetWorld()->GetTimerManager();
  State->bRolloverTimerExists = Timers.TimerExists(TimerHandler);
  State->bRolloverTimerPaused = State->bRolloverTimerExists && Timers.IsTimerPaused(TimerHandler);
  State->RolloverTimerRemaining = State->bRolloverTimerExists ? Timers.GetTimerRemaining(TimerHandler) : 0.0f;
  State->RolloverTimerRate = State->bRolloverTimerExists ? Timers.GetTimerRate(TimerHandler) : 0.0f;
  if (State->bRolloverTimerExists &&
      (!FMath::IsFinite(State->RolloverTimerRemaining) || State->RolloverTimerRemaining < 0.0f ||
       !FMath::IsFinite(State->RolloverTimerRate) || State->RolloverTimerRate <= 0.0f))
    return false;
  State->SetupTag = Movement->VehicleSetupTag;
  State->FixedDeltaSeconds = Settings.FixedDeltaSeconds.GetValue();
  State->MaxSubstepDeltaTime = Settings.MaxSubstepDeltaTime;
  State->MaxSubsteps = Settings.MaxSubsteps;
  State->bSubstepping = Settings.bSubstepping;
  State->DriveIdentity = Movement->PVehicleDrive;
  State->CapturedFrame = FCarlaEngine::GetFrameCounter();
  for (const TCHAR *Name : CheckpointFloatFields)
    State->MovementFloats.Add(FindFProperty<FFloatProperty>(Movement->GetClass(), Name)->GetPropertyValue_InContainer(Movement));
  for (const TCHAR *Name : CheckpointBoolFields)
    State->MovementBools.Add(FindFProperty<FBoolProperty>(Movement->GetClass(), Name)->GetPropertyValue_InContainer(Movement));

  FBodyInstance *Body = Root->GetBodyInstance();
  if (!Body || !Body->IsValidBodyInstance()) return false;
  bool Captured = false;
  FPhysicsCommand::ExecuteRead(Body->ActorHandle, [&](const FPhysicsActorHandle &Actor)
  {
    auto *Drive = Movement->PVehicleDrive;
    auto *RigidActor = FPhysicsInterface::GetPxRigidActor_AssumesLocked(Actor);
    auto *BodyActor = RigidActor ? RigidActor->is<physx::PxRigidDynamic>() : nullptr;
    if (!BodyActor || Drive->getRigidDynamicActor() != BodyActor) return;
    auto *Scene = BodyActor->getScene();
    if (!Scene) return;
    State->BodyPose = BodyActor->getGlobalPose();
    State->BodyLinearVelocity = BodyActor->getLinearVelocity();
    State->BodyAngularVelocity = BodyActor->getAngularVelocity();
    State->PhysXSceneFlags = static_cast<uint32>(Scene->getFlags());
    State->bAwake = !BodyActor->isSleeping();
    State->WakeCounter = BodyActor->getWakeCounter();
    State->Drive = Drive->mDriveDynData;
    auto &Wheels = Movement->PVehicle->mWheelsDynData;
    const auto *WheelBlocks = Wheels.getWheel4DynData();
    if (!WheelBlocks) return;
    for (physx::PxU32 I = 0; I < Wheels.getNbWheelRotationSpeed(); ++I)
    {
      const auto &Block = WheelBlocks[I / 4];
      const auto Slot = I % 4;
      State->Wheels.Add({Block.mWheelSpeeds[Slot], Block.mCorrectedWheelSpeeds[Slot],
                         Block.mWheelRotationAngles[Slot], Block.mTireLowForwardSpeedTimers[Slot],
                         Block.mTireLowSideSpeedTimers[Slot], Block.mJounces[Slot]});
    }
    Captured = true;
  });
  if (!Captured) return false;
  PhysicsCheckpoint = MoveTemp(State);
  return true;
#endif
}

bool ACarlaWheeledVehicle::ExportPhysicsCheckpoint(carla::rpc::PortableVehicleCheckpoint &Out) const
{
#if !CARLA_CHECKPOINT_PHYSX_WHEEL_INTERNALS
  return false;
#else
  if (!PhysicsCheckpoint) return false;
  const auto &State = *PhysicsCheckpoint;
  carla::rpc::PortableVehicleCheckpoint Data;
  Data.capture_frame = State.CapturedFrame;
  Data.physx_scene_flags = State.PhysXSceneFlags;
  Data.fixed_delta_seconds = State.FixedDeltaSeconds;
  Data.max_substep_delta_time = State.MaxSubstepDeltaTime;
  Data.max_substeps = State.MaxSubsteps;
  Data.substepping = State.bSubstepping;
  Data.physics_configuration = State.PhysicsConfiguration;
  const auto &Position = State.Transform.GetLocation();
  const auto &Rotation = State.Transform.GetRotation();
  const auto &Scale = State.Transform.GetScale3D();
  Data.position_cm = {Position.X, Position.Y, Position.Z};
  Data.rotation_quaternion = {Rotation.X, Rotation.Y, Rotation.Z, Rotation.W};
  Data.scale = {Scale.X, Scale.Y, Scale.Z};
  Data.linear_velocity_cm_s = {State.LinearVelocity.X, State.LinearVelocity.Y, State.LinearVelocity.Z};
  Data.angular_velocity_rad_s = {State.AngularVelocity.X, State.AngularVelocity.Y, State.AngularVelocity.Z};
  Data.awake = State.bAwake;
  Data.wake_counter = State.WakeCounter;
  Data.input_control = carla::rpc::VehicleControl(State.InputControl);
  Data.last_control = carla::rpc::VehicleControl(State.LastControl);
  Data.input_priority = static_cast<uint32>(State.InputPriority);
  Data.light_state = carla::rpc::VehicleLightState(State.LightState).GetLightStateAsValue();
  Data.rollover_tracker = State.RolloverTracker;
  Data.failure_state = static_cast<uint32>(State.FailureState);
  Data.rollover_timer_exists = State.bRolloverTimerExists;
  Data.rollover_timer_paused = State.bRolloverTimerPaused;
  Data.rollover_timer_remaining = State.RolloverTimerRemaining;
  Data.rollover_timer_rate = State.RolloverTimerRate;
  const auto &Drive = State.Drive;
  Data.drive.analog_inputs.assign(Drive.mControlAnalogVals,
                                  Drive.mControlAnalogVals + physx::PxVehicleDriveDynData::eMAX_NB_ANALOG_INPUTS);
  Data.drive.use_auto_gears = Drive.mUseAutoGears;
  Data.drive.gear_up = Drive.mGearUpPressed;
  Data.drive.gear_down = Drive.mGearDownPressed;
  Data.drive.current_gear = Drive.mCurrentGear;
  Data.drive.target_gear = Drive.mTargetGear;
  Data.drive.engine_speed = Drive.mEnginespeed;
  Data.drive.gear_switch_time = Drive.mGearSwitchTime;
  Data.drive.auto_box_switch_time = Drive.mAutoBoxSwitchTime;
  for (const auto &Wheel : State.Wheels)
    Data.wheels.push_back({Wheel.Speed, Wheel.CorrectedSpeed, Wheel.Angle,
        Wheel.LowForwardSpeedTimer, Wheel.LowSideSpeedTimer, Wheel.Jounce});
  for (float Value : State.MovementFloats) Data.movement_floats.push_back(Value);
  for (bool Value : State.MovementBools) Data.movement_bools.push_back(Value ? 1u : 0u);
  Data.body_position_physx = {State.BodyPose.p.x, State.BodyPose.p.y, State.BodyPose.p.z};
  Data.body_rotation_quaternion = {State.BodyPose.q.x, State.BodyPose.q.y, State.BodyPose.q.z, State.BodyPose.q.w};
  Data.body_linear_velocity_physx = {State.BodyLinearVelocity.x, State.BodyLinearVelocity.y, State.BodyLinearVelocity.z};
  Data.body_angular_velocity_physx = {State.BodyAngularVelocity.x, State.BodyAngularVelocity.y, State.BodyAngularVelocity.z};
  Data.observer_velocity_m_s = {State.ObserverVelocity.X, State.ObserverVelocity.Y, State.ObserverVelocity.Z};
  if (!Data.HasValidShape()) return false;
  Out = std::move(Data);
  return true;
#endif
}

bool ACarlaWheeledVehicle::RestorePhysicsCheckpoint(const FEpisodeSettings &Settings, FVector &ObserverVelocity)
{
#if !CARLA_CHECKPOINT_PHYSX_WHEEL_INTERNALS
  return false;
#else
  auto *Movement = GetVehicleMovementComponent();
  auto *Root = Cast<UPrimitiveComponent>(GetRootComponent());
  if (!PhysicsCheckpoint || !Settings.bSynchronousMode || !Settings.FixedDeltaSeconds.IsSet() ||
      Settings.FixedDeltaSeconds.GetValue() != PhysicsCheckpoint->FixedDeltaSeconds ||
      Settings.MaxSubstepDeltaTime != PhysicsCheckpoint->MaxSubstepDeltaTime ||
      Settings.MaxSubsteps != PhysicsCheckpoint->MaxSubsteps ||
      Settings.bSubstepping != PhysicsCheckpoint->bSubstepping ||
      !Movement || !Movement->UpdatedComponent || !Root || !GetWorld() || !Root->IsSimulatingPhysics() ||
      !Movement->PVehicleDrive || !Movement->PVehicle ||
      Movement->PVehicleDrive != PhysicsCheckpoint->DriveIdentity ||
      Movement->VehicleSetupTag != PhysicsCheckpoint->SetupTag ||
      Movement->PVehicle->mWheelsDynData.getNbWheelRotationSpeed() !=
        static_cast<physx::PxU32>(PhysicsCheckpoint->Wheels.Num()) ||
      !Movement->PVehicle->mWheelsDynData.getWheel4DynData() ||
      !Cast<UDefaultMovementComponent>(BaseMovementComponent) ||
      bAckermannControlActive || VelocityControl->IsActive() ||
      Movement->bUseRVOAvoidance || !CheckpointPropertiesExist(Movement) ||
      !Root->GetBodyInstance() || !Root->GetBodyInstance()->IsValidBodyInstance())
    return false;
  for (const auto &Door : ConstraintDoor)
    if (Door.Value && Door.Value->IsSimulatingPhysics()) return false;

  // Check the physics actor before applying the transform and input state.
  // Recheck under the write lock below: this preflight is not a transaction.
  bool BodyMatchesDrive = false;
  FPhysicsCommand::ExecuteRead(Root->GetBodyInstance()->ActorHandle, [&](const FPhysicsActorHandle &Actor)
  {
    auto *RigidActor = FPhysicsInterface::GetPxRigidActor_AssumesLocked(Actor);
    auto *BodyActor = RigidActor ? RigidActor->is<physx::PxRigidDynamic>() : nullptr;
    BodyMatchesDrive = BodyActor && Movement->PVehicleDrive->getRigidDynamicActor() == BodyActor;
  });
  if (!BodyMatchesDrive) return false;
  if (CheckpointPhysicsConfiguration(*this) != PhysicsCheckpoint->PhysicsConfiguration) return false;

  if (!SetActorTransform(PhysicsCheckpoint->Transform, false, nullptr, ETeleportType::TeleportPhysics))
    return false;
  Root->SetPhysicsLinearVelocity(PhysicsCheckpoint->LinearVelocity);
  Root->SetPhysicsAngularVelocityInRadians(PhysicsCheckpoint->AngularVelocity);
  InputControl.Control = PhysicsCheckpoint->InputControl;
  InputControl.Priority = PhysicsCheckpoint->InputPriority;
  LastAppliedControl = PhysicsCheckpoint->LastControl;
  RolloverBehaviorTracker = PhysicsCheckpoint->RolloverTracker;
  FailureState = PhysicsCheckpoint->FailureState;
  int32 Index = 0;
  for (const TCHAR *Name : CheckpointFloatFields)
    FindFProperty<FFloatProperty>(Movement->GetClass(), Name)->SetPropertyValue_InContainer(Movement, PhysicsCheckpoint->MovementFloats[Index++]);
  Index = 0;
  for (const TCHAR *Name : CheckpointBoolFields)
    FindFProperty<FBoolProperty>(Movement->GetClass(), Name)->SetPropertyValue_InContainer(Movement, PhysicsCheckpoint->MovementBools[Index++]);

  bool Restored = false;
  FPhysicsCommand::ExecuteWrite(Root->GetBodyInstance()->ActorHandle, [&](const FPhysicsActorHandle &Actor)
  {
    auto *RigidActor = FPhysicsInterface::GetPxRigidActor_AssumesLocked(Actor);
    auto *BodyActor = RigidActor ? RigidActor->is<physx::PxRigidDynamic>() : nullptr;
    if (!BodyActor || Movement->PVehicleDrive->getRigidDynamicActor() != BodyActor) return;
    // Restore the exact PhysX values after updating the Unreal component. The
    // component setters above can round-trip through a different transform
    // representation and do not expose the underlying actor pose directly.
    BodyActor->setGlobalPose(PhysicsCheckpoint->BodyPose, false);
    BodyActor->setLinearVelocity(PhysicsCheckpoint->BodyLinearVelocity, false);
    BodyActor->setAngularVelocity(PhysicsCheckpoint->BodyAngularVelocity, false);
    Movement->PVehicleDrive->mDriveDynData = PhysicsCheckpoint->Drive;
    auto &Wheels = Movement->PVehicle->mWheelsDynData;
    auto *WheelBlocks = Wheels.getWheel4DynData();
    for (int32 I = 0; I < PhysicsCheckpoint->Wheels.Num(); ++I)
    {
      auto &Block = WheelBlocks[I / 4];
      const int32 Slot = I % 4;
      const auto &Saved = PhysicsCheckpoint->Wheels[I];
      Block.mWheelSpeeds[Slot] = Saved.Speed;
      Block.mCorrectedWheelSpeeds[Slot] = Saved.CorrectedSpeed;
      Block.mWheelRotationAngles[Slot] = Saved.Angle;
      Block.mTireLowForwardSpeedTimers[Slot] = Saved.LowForwardSpeedTimer;
      Block.mTireLowSideSpeedTimers[Slot] = Saved.LowSideSpeedTimer;
      Block.mJounces[Slot] = Saved.Jounce;
    }
    if (PhysicsCheckpoint->bAwake)
    {
      BodyActor->wakeUp();
      BodyActor->setWakeCounter(PhysicsCheckpoint->WakeCounter);
    }
    else
    {
      BodyActor->putToSleep();
    }
    Restored = true;
  });
  if (Restored)
  {
    // WorldObserver differentiates velocity against this actor-info cache.
    // Restore it before the next tick so reported acceleration starts from
    // the checkpoint, rather than the end of the previous rollout.
    ObserverVelocity = PhysicsCheckpoint->ObserverVelocity;
    auto &Timers = GetWorld()->GetTimerManager();
    Timers.ClearTimer(TimerHandler);
    if (PhysicsCheckpoint->bRolloverTimerExists)
    {
      Timers.SetTimer(TimerHandler, this, &ACarlaWheeledVehicle::SetRolloverFlag,
                      PhysicsCheckpoint->RolloverTimerRate, false, PhysicsCheckpoint->RolloverTimerRemaining);
      if (PhysicsCheckpoint->bRolloverTimerPaused) Timers.PauseTimer(TimerHandler);
    }
    SetVehicleLightState(PhysicsCheckpoint->LightState);
  }
  return Restored;
#endif
}

void ACarlaWheeledVehicle::SetWheelCollision(UWheeledVehicleMovementComponent4W *Vehicle4W,
    const FVehiclePhysicsControl &PhysicsControl ) {

  #ifdef WHEEL_SWEEP_ENABLED
    const bool IsBike = IsTwoWheeledVehicle();

    if (IsBike)
      return;

    const bool IsEqual = Vehicle4W->UseSweepWheelCollision == PhysicsControl.UseSweepWheelCollision;

    if (IsEqual)
      return;

    Vehicle4W->UseSweepWheelCollision = PhysicsControl.UseSweepWheelCollision;

  #else

    if (PhysicsControl.UseSweepWheelCollision)
      UE_LOG(LogCarla, Warning, TEXT("Error: Sweep for wheel collision is not available. \
      Make sure you have installed the required patch.") );

  #endif

}

void ACarlaWheeledVehicle::SetWheelCollisionNW(UWheeledVehicleMovementComponentNW *VehicleNW,
    const FVehiclePhysicsControl &PhysicsControl ) {

  #ifdef WHEEL_SWEEP_ENABLED
    const bool IsEqual = VehicleNW->UseSweepWheelCollision == PhysicsControl.UseSweepWheelCollision;

    if (IsEqual)
      return;

    VehicleNW->UseSweepWheelCollision = PhysicsControl.UseSweepWheelCollision;

  #else

    if (PhysicsControl.UseSweepWheelCollision)
      UE_LOG(LogCarla, Warning, TEXT("Error: Sweep for wheel collision is not available. \
      Make sure you have installed the required patch.") );

  #endif
}

void ACarlaWheeledVehicle::BeginPlay()
{
  Super::BeginPlay();

  UDefaultMovementComponent::CreateDefaultMovementComponent(this);

  // Get constraint components and their initial transforms
  FTransform ActorInverseTransform = GetActorTransform().Inverse();
  ConstraintsComponents.Empty();
  DoorComponentsTransform.Empty();
  ConstraintDoor.Empty();
  for (FName& ComponentName : ConstraintComponentNames)
  {
    UPhysicsConstraintComponent* ConstraintComponent =
        Cast<UPhysicsConstraintComponent>(GetDefaultSubobjectByName(ComponentName));
    if (ConstraintComponent)
    {
      UPrimitiveComponent* DoorComponent = Cast<UPrimitiveComponent>(
          GetDefaultSubobjectByName(ConstraintComponent->ComponentName1.ComponentName));
      if(DoorComponent)
      {
        UE_LOG(LogCarla, Warning, TEXT("Door name: %s"), *(DoorComponent->GetName()));
        FTransform ComponentWorldTransform = DoorComponent->GetComponentTransform();
        FTransform RelativeTransform = ComponentWorldTransform * ActorInverseTransform;
        DoorComponentsTransform.Add(DoorComponent, RelativeTransform);
        ConstraintDoor.Add(ConstraintComponent, DoorComponent);
        ConstraintsComponents.Add(ConstraintComponent);
        ConstraintComponent->TermComponentConstraint();
      }
      else
      {
        UE_LOG(LogCarla, Error, TEXT("Missing component for constraint: %s"), *(ConstraintComponent->GetName()));
      }
    }
  }
  ResetConstraints();

  // get collision disable constraints (used to prevent doors from colliding with each other)
  CollisionDisableConstraints.Empty();
  TArray<UPhysicsConstraintComponent*> Constraints;
  GetComponents(Constraints);
  for (UPhysicsConstraintComponent* Constraint : Constraints)
  {
    if (!ConstraintsComponents.Contains(Constraint))
    {
      UPrimitiveComponent* CollisionDisabledComponent1 = Cast<UPrimitiveComponent>(
          GetDefaultSubobjectByName(Constraint->ComponentName1.ComponentName));
      UPrimitiveComponent* CollisionDisabledComponent2 = Cast<UPrimitiveComponent>(
          GetDefaultSubobjectByName(Constraint->ComponentName2.ComponentName));
      if (CollisionDisabledComponent1)
      {
        CollisionDisableConstraints.Add(CollisionDisabledComponent1, Constraint);
      }
      if (CollisionDisabledComponent2)
      {
        CollisionDisableConstraints.Add(CollisionDisabledComponent2, Constraint);
      }
    }
  }

  float FrictionScale = 3.5f;

  UWheeledVehicleMovementComponent* MovementComponent = GetVehicleMovementComponent();

  if (MovementComponent)
  {
    check(MovementComponent != nullptr);

    // Setup Tire Configs with default value. This is needed to avoid getting
    // friction values of previously created TireConfigs for the same vehicle
    // blueprint.
    TArray<float> OriginalFrictions;
    OriginalFrictions.Init(FrictionScale, MovementComponent->Wheels.Num());
    SetWheelsFrictionScale(OriginalFrictions);

    // Check if it overlaps with a Friction trigger, if so, update the friction
    // scale.
    TArray<AActor *> OverlapActors;
    GetOverlappingActors(OverlapActors, AFrictionTrigger::StaticClass());
    for (const auto &Actor : OverlapActors)
    {
      AFrictionTrigger *FrictionTrigger = Cast<AFrictionTrigger>(Actor);
      if (FrictionTrigger)
      {
        FrictionScale = FrictionTrigger->Friction;
      }
    }

    // Set the friction scale to Wheel CDO and update wheel setups
    TArray<FWheelSetup> NewWheelSetups = MovementComponent->WheelSetups;
    for (const auto &WheelSetup : NewWheelSetups)
    {
      UVehicleWheel *Wheel = WheelSetup.WheelClass.GetDefaultObject();
      check(Wheel != nullptr);
    }

    MovementComponent->WheelSetups = NewWheelSetups;

    LastPhysicsControl = GetVehiclePhysicsControl();

    // Update physics in the Ackermann Controller
    AckermannController.UpdateVehiclePhysics(this);
  }

  AddReferenceToManager();
}

bool ACarlaWheeledVehicle::IsInVehicleRange(const FVector& Location) const
{
  TRACE_CPUPROFILER_EVENT_SCOPE(ACarlaWheeledVehicle::IsInVehicleRange);

  return FoliageBoundingBox.IsInside(Location);
}

void ACarlaWheeledVehicle::UpdateDetectionBox()
{
  const FTransform GlobalTransform = GetActorTransform();
  const FVector Vec { DetectionSize, DetectionSize, DetectionSize };
  FBox Box = FBox(-Vec, Vec);
  const FTransform NonScaledTransform(GlobalTransform.GetRotation(), GlobalTransform.GetLocation(), {1.0f, 1.0f, 1.0f});
  FoliageBoundingBox = Box.TransformBy(NonScaledTransform);
}

const TArray<int32> ACarlaWheeledVehicle::GetFoliageInstancesCloseToVehicle(const UInstancedStaticMeshComponent* Component) const
{  
  TRACE_CPUPROFILER_EVENT_SCOPE(ACarlaWheeledVehicle::GetFoliageInstancesCloseToVehicle);
  return Component->GetInstancesOverlappingBox(FoliageBoundingBox);
}

FBox ACarlaWheeledVehicle::GetDetectionBox() const
{  
  TRACE_CPUPROFILER_EVENT_SCOPE(ACarlaWheeledVehicle::GetDetectionBox);
  return FoliageBoundingBox;
}

float ACarlaWheeledVehicle::GetDetectionSize() const
{
  return DetectionSize;
}

void ACarlaWheeledVehicle::DrawFoliageBoundingBox() const
{
  const FVector& Center = FoliageBoundingBox.GetCenter();
  const FVector& Extent = FoliageBoundingBox.GetExtent();
  const FQuat& Rotation = GetActorQuat();
  DrawDebugBox(GetWorld(), Center, Extent, Rotation, FColor::Magenta, false, 0.0f, 0, 5.0f);
}

FBoxSphereBounds ACarlaWheeledVehicle::GetBoxSphereBounds() const
{
  ALargeMapManager* LargeMap = UCarlaStatics::GetLargeMapManager(GetWorld());
  if (LargeMap)
  {
    FTransform GlobalTransform = LargeMap->LocalToGlobalTransform(GetActorTransform());
    return VehicleBounds->CalcBounds(GlobalTransform);
  }
  return VehicleBounds->CalcBounds(GetActorTransform());
}

void ACarlaWheeledVehicle::AdjustVehicleBounds()
{
  FBoundingBox BoundingBox = UBoundingBoxCalculator::GetVehicleBoundingBox(this);

  const FTransform& CompToWorldTransform = RootComponent->GetComponentTransform();
  const FRotator Rotation = CompToWorldTransform.GetRotation().Rotator();
  const FVector Translation = CompToWorldTransform.GetLocation();
  const FVector Scale = CompToWorldTransform.GetScale3D();

  // Invert BB origin to local space
  BoundingBox.Origin -= Translation;
  BoundingBox.Origin = Rotation.UnrotateVector(BoundingBox.Origin);
  BoundingBox.Origin /= Scale;

  // Prepare Box Collisions
  FTransform Transform;
  Transform.SetTranslation(BoundingBox.Origin);
  VehicleBounds->SetRelativeTransform(Transform);
  VehicleBounds->SetBoxExtent(BoundingBox.Extent);
}

// =============================================================================
// -- Get functions ------------------------------------------------------------
// =============================================================================

float ACarlaWheeledVehicle::GetVehicleForwardSpeed() const
{
  return BaseMovementComponent->GetVehicleForwardSpeed();
}

FVector ACarlaWheeledVehicle::GetVehicleOrientation() const
{
  return GetVehicleTransform().GetRotation().GetForwardVector();
}

int32 ACarlaWheeledVehicle::GetVehicleCurrentGear() const
{
    return BaseMovementComponent->GetVehicleCurrentGear();
}

FTransform ACarlaWheeledVehicle::GetVehicleBoundingBoxTransform() const
{
  return VehicleBounds->GetRelativeTransform();
}

FVector ACarlaWheeledVehicle::GetVehicleBoundingBoxExtent() const
{
  return VehicleBounds->GetScaledBoxExtent();
}

float ACarlaWheeledVehicle::GetMaximumSteerAngle() const
{
  const auto &Wheels = GetVehicleMovementComponent()->Wheels;
  check(Wheels.Num() > 0);
  const auto *FrontWheel = Wheels[0];
  check(FrontWheel != nullptr);
  return FrontWheel->SteerAngle;
}

// =============================================================================
// -- Set functions ------------------------------------------------------------
// =============================================================================

void ACarlaWheeledVehicle::FlushVehicleControl()
{
  if (bAckermannControlActive) {
    AckermannController.UpdateVehicleState(this);
    AckermannController.RunLoop(InputControl.Control);
  }

  BaseMovementComponent->ProcessControl(InputControl.Control);
  InputControl.Control.bReverse = InputControl.Control.Gear < 0;
  LastAppliedControl = InputControl.Control;
  InputControl.Priority = EVehicleInputPriority::INVALID;
}

void ACarlaWheeledVehicle::SetThrottleInput(const float Value)
{
  FVehicleControl Control = InputControl.Control;
  Control.Throttle = Value;
  ApplyVehicleControl(Control, EVehicleInputPriority::User);
}

void ACarlaWheeledVehicle::SetSteeringInput(const float Value)
{
  FVehicleControl Control = InputControl.Control;
  Control.Steer = Value;
  ApplyVehicleControl(Control, EVehicleInputPriority::User);
}

void ACarlaWheeledVehicle::SetBrakeInput(const float Value)
{
  FVehicleControl Control = InputControl.Control;
  Control.Brake = Value;
  ApplyVehicleControl(Control, EVehicleInputPriority::User);
}

void ACarlaWheeledVehicle::SetReverse(const bool Value)
{
  FVehicleControl Control = InputControl.Control;
  Control.bReverse = Value;
  ApplyVehicleControl(Control, EVehicleInputPriority::User);
}

void ACarlaWheeledVehicle::SetHandbrakeInput(const bool Value)
{
  FVehicleControl Control = InputControl.Control;
  Control.bHandBrake = Value;
  ApplyVehicleControl(Control, EVehicleInputPriority::User);
}

TArray<float> ACarlaWheeledVehicle::GetWheelsFrictionScale()
{

  UWheeledVehicleMovementComponent* Movement = GetVehicleMovement();
  TArray<float> WheelsFrictionScale;
  if (Movement)
  {
    check(Movement != nullptr);

    for (auto &Wheel : Movement->Wheels)
    {
      WheelsFrictionScale.Add(Wheel->TireConfig->GetFrictionScale());
    }
  }
  return WheelsFrictionScale;
}

void ACarlaWheeledVehicle::SetWheelsFrictionScale(TArray<float> &WheelsFrictionScale)
{
  PhysicsCheckpoint.Reset();

  UWheeledVehicleMovementComponent* Movement = GetVehicleMovement();
  if (Movement)
  {
    check(Movement != nullptr);
    check(Movement->Wheels.Num() == WheelsFrictionScale.Num());

    for (int32 i = 0; i < Movement->Wheels.Num(); ++i)
    {
      Movement->Wheels[i]->TireConfig->SetFrictionScale(WheelsFrictionScale[i]);
    }
  }
}

FVehiclePhysicsControl ACarlaWheeledVehicle::GetVehiclePhysicsControl() const
{
  FVehiclePhysicsControl PhysicsControl;

  if (!bIsNWVehicle) {
    UWheeledVehicleMovementComponent4W *Vehicle4W = Cast<UWheeledVehicleMovementComponent4W>(
          GetVehicleMovement());
    check(Vehicle4W != nullptr);

    // Engine Setup
    PhysicsControl.TorqueCurve = Vehicle4W->EngineSetup.TorqueCurve.EditorCurveData;
    PhysicsControl.MaxRPM = Vehicle4W->EngineSetup.MaxRPM;
    PhysicsControl.MOI = Vehicle4W->EngineSetup.MOI;
    PhysicsControl.DampingRateFullThrottle = Vehicle4W->EngineSetup.DampingRateFullThrottle;
    PhysicsControl.DampingRateZeroThrottleClutchEngaged =
        Vehicle4W->EngineSetup.DampingRateZeroThrottleClutchEngaged;
    PhysicsControl.DampingRateZeroThrottleClutchDisengaged =
        Vehicle4W->EngineSetup.DampingRateZeroThrottleClutchDisengaged;

    // Transmission Setup
    PhysicsControl.bUseGearAutoBox = Vehicle4W->TransmissionSetup.bUseGearAutoBox;
    PhysicsControl.GearSwitchTime = Vehicle4W->TransmissionSetup.GearSwitchTime;
    PhysicsControl.ClutchStrength = Vehicle4W->TransmissionSetup.ClutchStrength;
    PhysicsControl.FinalRatio = Vehicle4W->TransmissionSetup.FinalRatio;

    TArray<FGearPhysicsControl> ForwardGears;

    for (const auto &Gear : Vehicle4W->TransmissionSetup.ForwardGears)
    {
      FGearPhysicsControl GearPhysicsControl;

      GearPhysicsControl.Ratio = Gear.Ratio;
      GearPhysicsControl.UpRatio = Gear.UpRatio;
      GearPhysicsControl.DownRatio = Gear.DownRatio;

      ForwardGears.Add(GearPhysicsControl);
    }

    PhysicsControl.ForwardGears = ForwardGears;

    // Vehicle Setup
    PhysicsControl.Mass = Vehicle4W->Mass;
    PhysicsControl.DragCoefficient = Vehicle4W->DragCoefficient;

    // Center of mass offset (Center of mass is always zero vector in local
    // position)
    UPrimitiveComponent *UpdatedPrimitive = Cast<UPrimitiveComponent>(Vehicle4W->UpdatedComponent);
    check(UpdatedPrimitive != nullptr);

    PhysicsControl.CenterOfMass = UpdatedPrimitive->BodyInstance.COMNudge;

    // Transmission Setup
    PhysicsControl.SteeringCurve = Vehicle4W->SteeringCurve.EditorCurveData;

    // Wheels Setup
    TArray<FWheelPhysicsControl> Wheels;

    for (int32 i = 0; i < Vehicle4W->WheelSetups.Num(); ++i)
    {
      FWheelPhysicsControl PhysicsWheel;

      PxVehicleWheelData PWheelData = Vehicle4W->PVehicle->mWheelsSimData.getWheelData(i);
      PhysicsWheel.DampingRate = Cm2ToM2(PWheelData.mDampingRate);
      PhysicsWheel.MaxSteerAngle = FMath::RadiansToDegrees(PWheelData.mMaxSteer);
      PhysicsWheel.Radius = PWheelData.mRadius;
      PhysicsWheel.MaxBrakeTorque = Cm2ToM2(PWheelData.mMaxBrakeTorque);
      PhysicsWheel.MaxHandBrakeTorque = Cm2ToM2(PWheelData.mMaxHandBrakeTorque);

      PxVehicleTireData PTireData = Vehicle4W->PVehicle->mWheelsSimData.getTireData(i);
      PhysicsWheel.LatStiffMaxLoad = PTireData.mLatStiffX;
      PhysicsWheel.LatStiffValue = PTireData.mLatStiffY;
      PhysicsWheel.LongStiffValue = PTireData.mLongitudinalStiffnessPerUnitGravity;

      PhysicsWheel.TireFriction = Vehicle4W->Wheels[i]->TireConfig->GetFrictionScale();
      PhysicsWheel.Position = Vehicle4W->Wheels[i]->Location;

      Wheels.Add(PhysicsWheel);
    }

    PhysicsControl.Wheels = Wheels;

  } else {
    UWheeledVehicleMovementComponentNW *VehicleNW = Cast<UWheeledVehicleMovementComponentNW>(
      GetVehicleMovement());

    check(VehicleNW != nullptr);

    // Engine Setup
    PhysicsControl.TorqueCurve = VehicleNW->EngineSetup.TorqueCurve.EditorCurveData;
    PhysicsControl.MaxRPM = VehicleNW->EngineSetup.MaxRPM;
    PhysicsControl.MOI = VehicleNW->EngineSetup.MOI;
    PhysicsControl.DampingRateFullThrottle = VehicleNW->EngineSetup.DampingRateFullThrottle;
    PhysicsControl.DampingRateZeroThrottleClutchEngaged =
        VehicleNW->EngineSetup.DampingRateZeroThrottleClutchEngaged;
    PhysicsControl.DampingRateZeroThrottleClutchDisengaged =
        VehicleNW->EngineSetup.DampingRateZeroThrottleClutchDisengaged;

    // Transmission Setup
    PhysicsControl.bUseGearAutoBox = VehicleNW->TransmissionSetup.bUseGearAutoBox;
    PhysicsControl.GearSwitchTime = VehicleNW->TransmissionSetup.GearSwitchTime;
    PhysicsControl.ClutchStrength = VehicleNW->TransmissionSetup.ClutchStrength;
    PhysicsControl.FinalRatio = VehicleNW->TransmissionSetup.FinalRatio;

    TArray<FGearPhysicsControl> ForwardGears;

    for (const auto &Gear : VehicleNW->TransmissionSetup.ForwardGears)
    {
      FGearPhysicsControl GearPhysicsControl;

      GearPhysicsControl.Ratio = Gear.Ratio;
      GearPhysicsControl.UpRatio = Gear.UpRatio;
      GearPhysicsControl.DownRatio = Gear.DownRatio;

      ForwardGears.Add(GearPhysicsControl);
    }

    PhysicsControl.ForwardGears = ForwardGears;

    // VehicleNW Setup
    PhysicsControl.Mass = VehicleNW->Mass;
    PhysicsControl.DragCoefficient = VehicleNW->DragCoefficient;

    // Center of mass offset (Center of mass is always zero vector in local
    // position)
    UPrimitiveComponent *UpdatedPrimitive = Cast<UPrimitiveComponent>(VehicleNW->UpdatedComponent);
    check(UpdatedPrimitive != nullptr);

    PhysicsControl.CenterOfMass = UpdatedPrimitive->BodyInstance.COMNudge;

    // Transmission Setup
    PhysicsControl.SteeringCurve = VehicleNW->SteeringCurve.EditorCurveData;

    // Wheels Setup
    TArray<FWheelPhysicsControl> Wheels;

    for (int32 i = 0; i < VehicleNW->WheelSetups.Num(); ++i)
    {
      FWheelPhysicsControl PhysicsWheel;

      PxVehicleWheelData PWheelData = VehicleNW->PVehicle->mWheelsSimData.getWheelData(i);
      PhysicsWheel.DampingRate = Cm2ToM2(PWheelData.mDampingRate);
      PhysicsWheel.MaxSteerAngle = FMath::RadiansToDegrees(PWheelData.mMaxSteer);
      PhysicsWheel.Radius = PWheelData.mRadius;
      PhysicsWheel.MaxBrakeTorque = Cm2ToM2(PWheelData.mMaxBrakeTorque);
      PhysicsWheel.MaxHandBrakeTorque = Cm2ToM2(PWheelData.mMaxHandBrakeTorque);

      PxVehicleTireData PTireData = VehicleNW->PVehicle->mWheelsSimData.getTireData(i);
      PhysicsWheel.LatStiffMaxLoad = PTireData.mLatStiffX;
      PhysicsWheel.LatStiffValue = PTireData.mLatStiffY;
      PhysicsWheel.LongStiffValue = PTireData.mLongitudinalStiffnessPerUnitGravity;

      PhysicsWheel.TireFriction = VehicleNW->Wheels[i]->TireConfig->GetFrictionScale();
      PhysicsWheel.Position = VehicleNW->Wheels[i]->Location;

      Wheels.Add(PhysicsWheel);
    }

    PhysicsControl.Wheels = Wheels;

  }
  return PhysicsControl;
}

FVehicleLightState ACarlaWheeledVehicle::GetVehicleLightState() const
{
  return InputControl.LightState;
}

void ACarlaWheeledVehicle::RestoreVehiclePhysicsControl()
{
  ApplyVehiclePhysicsControl(LastPhysicsControl);
}

void ACarlaWheeledVehicle::ApplyVehiclePhysicsControl(const FVehiclePhysicsControl &PhysicsControl)
{
  PhysicsCheckpoint.Reset();
  LastPhysicsControl = PhysicsControl;
  if (!bIsNWVehicle) {
    UWheeledVehicleMovementComponent4W *Vehicle4W = Cast<UWheeledVehicleMovementComponent4W>(
          GetVehicleMovement());
    check(Vehicle4W != nullptr);

    

    // Engine Setup
    Vehicle4W->EngineSetup.TorqueCurve.EditorCurveData = PhysicsControl.TorqueCurve;
    Vehicle4W->EngineSetup.MaxRPM = PhysicsControl.MaxRPM;

    Vehicle4W->EngineSetup.MOI = PhysicsControl.MOI;

    Vehicle4W->EngineSetup.DampingRateFullThrottle = PhysicsControl.DampingRateFullThrottle;
    Vehicle4W->EngineSetup.DampingRateZeroThrottleClutchEngaged =
        PhysicsControl.DampingRateZeroThrottleClutchEngaged;
    Vehicle4W->EngineSetup.DampingRateZeroThrottleClutchDisengaged =
        PhysicsControl.DampingRateZeroThrottleClutchDisengaged;

    // Transmission Setup
    Vehicle4W->TransmissionSetup.bUseGearAutoBox = PhysicsControl.bUseGearAutoBox;
    Vehicle4W->TransmissionSetup.GearSwitchTime = PhysicsControl.GearSwitchTime;
    Vehicle4W->TransmissionSetup.ClutchStrength = PhysicsControl.ClutchStrength;
    Vehicle4W->TransmissionSetup.FinalRatio = PhysicsControl.FinalRatio;

    TArray<FVehicleGearData> ForwardGears;

    for (const auto &Gear : PhysicsControl.ForwardGears)
    {
      FVehicleGearData GearData;

      GearData.Ratio = Gear.Ratio;
      GearData.UpRatio = Gear.UpRatio;
      GearData.DownRatio = Gear.DownRatio;

      ForwardGears.Add(GearData);
    }

    Vehicle4W->TransmissionSetup.ForwardGears = ForwardGears;

    // Vehicle Setup
    Vehicle4W->Mass = PhysicsControl.Mass;
    Vehicle4W->DragCoefficient = PhysicsControl.DragCoefficient;

    // Center of mass
    UPrimitiveComponent *UpdatedPrimitive = Cast<UPrimitiveComponent>(Vehicle4W->UpdatedComponent);
    check(UpdatedPrimitive != nullptr);

    UpdatedPrimitive->BodyInstance.COMNudge = PhysicsControl.CenterOfMass;

    // Transmission Setup
    Vehicle4W->SteeringCurve.EditorCurveData = PhysicsControl.SteeringCurve;

    // Wheels Setup
    const int PhysicsWheelsNum = PhysicsControl.Wheels.Num();
    if (PhysicsWheelsNum != 4)
    {
      UE_LOG(LogCarla, Error, TEXT("Number of WheelPhysicsControl is not 4."));
      return;
    }

    // Change, if required, the collision mode for wheels
    SetWheelCollision(Vehicle4W, PhysicsControl);

    TArray<FWheelSetup> NewWheelSetups = Vehicle4W->WheelSetups;

    for (int32 i = 0; i < PhysicsWheelsNum; ++i)
    {
      UVehicleWheel *Wheel = NewWheelSetups[i].WheelClass.GetDefaultObject();
      check(Wheel != nullptr);

      // Assigning new tire config
      Wheel->TireConfig = DuplicateObject<UTireConfig>(Wheel->TireConfig, nullptr);

      // Setting a new value to friction
      Wheel->TireConfig->SetFrictionScale(PhysicsControl.Wheels[i].TireFriction);
    }

    Vehicle4W->WheelSetups = NewWheelSetups;

    // Recreate Physics State for vehicle setup
    GetWorld()->GetPhysicsScene()->GetPxScene()->lockWrite();
    Vehicle4W->RecreatePhysicsState();
    GetWorld()->GetPhysicsScene()->GetPxScene()->unlockWrite();

    for (int32 i = 0; i < PhysicsWheelsNum; ++i)
    {
      PxVehicleWheelData PWheelData = Vehicle4W->PVehicle->mWheelsSimData.getWheelData(i);

      PWheelData.mRadius = PhysicsControl.Wheels[i].Radius;
      PWheelData.mMaxSteer = FMath::DegreesToRadians(PhysicsControl.Wheels[i].MaxSteerAngle);
      PWheelData.mDampingRate = M2ToCm2(PhysicsControl.Wheels[i].DampingRate);
      PWheelData.mMaxBrakeTorque = M2ToCm2(PhysicsControl.Wheels[i].MaxBrakeTorque);
      PWheelData.mMaxHandBrakeTorque = M2ToCm2(PhysicsControl.Wheels[i].MaxHandBrakeTorque);
      Vehicle4W->PVehicle->mWheelsSimData.setWheelData(i, PWheelData);

      PxVehicleTireData PTireData = Vehicle4W->PVehicle->mWheelsSimData.getTireData(i);
      PTireData.mLatStiffX = PhysicsControl.Wheels[i].LatStiffMaxLoad;
      PTireData.mLatStiffY = PhysicsControl.Wheels[i].LatStiffValue;
      PTireData.mLongitudinalStiffnessPerUnitGravity = PhysicsControl.Wheels[i].LongStiffValue;
      Vehicle4W->PVehicle->mWheelsSimData.setTireData(i, PTireData);
    }

    ResetConstraints();
  } else {
    UWheeledVehicleMovementComponentNW *VehicleNW = Cast<UWheeledVehicleMovementComponentNW>(
          GetVehicleMovement());

    check(VehicleNW != nullptr);

    // Engine Setup
    VehicleNW->EngineSetup.TorqueCurve.EditorCurveData = PhysicsControl.TorqueCurve;
    VehicleNW->EngineSetup.MaxRPM = PhysicsControl.MaxRPM;

    VehicleNW->EngineSetup.MOI = PhysicsControl.MOI;

    VehicleNW->EngineSetup.DampingRateFullThrottle = PhysicsControl.DampingRateFullThrottle;
    VehicleNW->EngineSetup.DampingRateZeroThrottleClutchEngaged =
        PhysicsControl.DampingRateZeroThrottleClutchEngaged;
    VehicleNW->EngineSetup.DampingRateZeroThrottleClutchDisengaged =
        PhysicsControl.DampingRateZeroThrottleClutchDisengaged;

    // Transmission Setup
    VehicleNW->TransmissionSetup.bUseGearAutoBox = PhysicsControl.bUseGearAutoBox;
    VehicleNW->TransmissionSetup.GearSwitchTime = PhysicsControl.GearSwitchTime;
    VehicleNW->TransmissionSetup.ClutchStrength = PhysicsControl.ClutchStrength;
    VehicleNW->TransmissionSetup.FinalRatio = PhysicsControl.FinalRatio;

    TArray<FVehicleNWGearData> ForwardGears;

    for (const auto &Gear : PhysicsControl.ForwardGears)
    {
      FVehicleNWGearData GearData;

      GearData.Ratio = Gear.Ratio;
      GearData.UpRatio = Gear.UpRatio;
      GearData.DownRatio = Gear.DownRatio;

      ForwardGears.Add(GearData);
    }

    VehicleNW->TransmissionSetup.ForwardGears = ForwardGears;

    // VehicleNW Setup
    VehicleNW->Mass = PhysicsControl.Mass;
    VehicleNW->DragCoefficient = PhysicsControl.DragCoefficient;

    // Center of mass
    UPrimitiveComponent *UpdatedPrimitive = Cast<UPrimitiveComponent>(VehicleNW->UpdatedComponent);
    check(UpdatedPrimitive != nullptr);

    UpdatedPrimitive->BodyInstance.COMNudge = PhysicsControl.CenterOfMass;

    // Transmission Setup
    VehicleNW->SteeringCurve.EditorCurveData = PhysicsControl.SteeringCurve;

    // Wheels Setup
    const int PhysicsWheelsNum = PhysicsControl.Wheels.Num();

    // Change, if required, the collision mode for wheels
    SetWheelCollisionNW(VehicleNW, PhysicsControl);

    TArray<FWheelSetup> NewWheelSetups = VehicleNW->WheelSetups;

    for (int32 i = 0; i < PhysicsWheelsNum; ++i)
    {
      UVehicleWheel *Wheel = NewWheelSetups[i].WheelClass.GetDefaultObject();
      check(Wheel != nullptr);

      // Assigning new tire config
      Wheel->TireConfig = DuplicateObject<UTireConfig>(Wheel->TireConfig, nullptr);

      // Setting a new value to friction
      Wheel->TireConfig->SetFrictionScale(PhysicsControl.Wheels[i].TireFriction);
    }

    VehicleNW->WheelSetups = NewWheelSetups;

    // Recreate Physics State for vehicle setup
    GetWorld()->GetPhysicsScene()->GetPxScene()->lockWrite();
    VehicleNW->RecreatePhysicsState();
    GetWorld()->GetPhysicsScene()->GetPxScene()->unlockWrite();

    for (int32 i = 0; i < PhysicsWheelsNum; ++i)
    {
      PxVehicleWheelData PWheelData = VehicleNW->PVehicle->mWheelsSimData.getWheelData(i);

      PWheelData.mRadius = PhysicsControl.Wheels[i].Radius;
      PWheelData.mMaxSteer = FMath::DegreesToRadians(PhysicsControl.Wheels[i].MaxSteerAngle);
      PWheelData.mDampingRate = M2ToCm2(PhysicsControl.Wheels[i].DampingRate);
      PWheelData.mMaxBrakeTorque = M2ToCm2(PhysicsControl.Wheels[i].MaxBrakeTorque);
      PWheelData.mMaxHandBrakeTorque = M2ToCm2(PhysicsControl.Wheels[i].MaxHandBrakeTorque);
      VehicleNW->PVehicle->mWheelsSimData.setWheelData(i, PWheelData);

      PxVehicleTireData PTireData = VehicleNW->PVehicle->mWheelsSimData.getTireData(i);
      PTireData.mLatStiffX = PhysicsControl.Wheels[i].LatStiffMaxLoad;
      PTireData.mLatStiffY = PhysicsControl.Wheels[i].LatStiffValue;
      PTireData.mLongitudinalStiffnessPerUnitGravity = PhysicsControl.Wheels[i].LongStiffValue;
      VehicleNW->PVehicle->mWheelsSimData.setTireData(i, PTireData);
    }

    ResetConstraints();

  }

  auto * Recorder = UCarlaStatics::GetRecorder(GetWorld());
  if (Recorder && Recorder->IsEnabled())
  {
    Recorder->AddPhysicsControl(*this);
  }

  // Update physics in the Ackermann Controller
  AckermannController.UpdateVehiclePhysics(this);
  
}

void ACarlaWheeledVehicle::ActivateVelocityControl(const FVector &Velocity)
{
  VelocityControl->Activate(Velocity);
}

void ACarlaWheeledVehicle::DeactivateVelocityControl()
{
  VelocityControl->Deactivate();
}

void ACarlaWheeledVehicle::ShowDebugTelemetry(bool Enabled)
{
  if (GetWorld()->GetFirstPlayerController())
  {
    ACarlaHUD* hud = Cast<ACarlaHUD>(GetWorld()->GetFirstPlayerController()->GetHUD());
    if (hud) {

      // Set/Unset the car movement component in HUD to show the temetry
      if (Enabled) {
        hud->AddDebugVehicleForTelemetry(GetVehicleMovementComponent());
      }
      else{
        if (hud->DebugVehicle == GetVehicleMovementComponent()) {
          hud->AddDebugVehicleForTelemetry(nullptr);
          GetVehicleMovementComponent()->StopTelemetry();
        }
      }

    }
    else {
      UE_LOG(LogCarla, Warning, TEXT("ACarlaWheeledVehicle::ShowDebugTelemetry:: Cannot find HUD for debug info"));
    }
  }
}

void ACarlaWheeledVehicle::SetVehicleLightState(const FVehicleLightState &LightState)
{
  if (LightState.Position != InputControl.LightState.Position ||
      LightState.LowBeam != InputControl.LightState.LowBeam ||
      LightState.HighBeam != InputControl.LightState.HighBeam ||
      LightState.Brake != InputControl.LightState.Brake ||
      LightState.RightBlinker != InputControl.LightState.RightBlinker ||
      LightState.LeftBlinker != InputControl.LightState.LeftBlinker ||
      LightState.Reverse != InputControl.LightState.Reverse ||
      LightState.Fog != InputControl.LightState.Fog ||
      LightState.Interior != InputControl.LightState.Interior ||
      LightState.Special1 != InputControl.LightState.Special1 ||
      LightState.Special2 != InputControl.LightState.Special2)
  {
    InputControl.LightState = LightState;
    RefreshLightState(LightState);
  }
}

void ACarlaWheeledVehicle::SetFailureState(const carla::rpc::VehicleFailureState &InFailureState)
{
  FailureState = InFailureState;
}

void ACarlaWheeledVehicle::SetCarlaMovementComponent(UBaseCarlaMovementComponent* MovementComponent)
{
  PhysicsCheckpoint.Reset();
  if (BaseMovementComponent)
  {
    BaseMovementComponent->DestroyComponent();
  }
  BaseMovementComponent = MovementComponent;
}

void ACarlaWheeledVehicle::SetWheelSteerDirection(EVehicleWheelLocation WheelLocation, float AngleInDeg) {

  if (bPhysicsEnabled == false)
  {
    check((uint8)WheelLocation >= 0)
    UVehicleAnimInstance *VehicleAnim = Cast<UVehicleAnimInstance>(GetMesh()->GetAnimInstance());
    check(VehicleAnim != nullptr)
    VehicleAnim->SetWheelRotYaw((uint8)WheelLocation, AngleInDeg);
  }
  else
  {
    UE_LOG(LogTemp, Warning, TEXT("Cannot set wheel steer direction. Physics are enabled."))
  }
}

float ACarlaWheeledVehicle::GetWheelSteerAngle(EVehicleWheelLocation WheelLocation) {

  check((uint8)WheelLocation >= 0)
  UVehicleAnimInstance *VehicleAnim = Cast<UVehicleAnimInstance>(GetMesh()->GetAnimInstance());
  check(VehicleAnim != nullptr)
  check(VehicleAnim->GetWheeledVehicleMovementComponent() != nullptr)

  if (bPhysicsEnabled == true)
  {
    return VehicleAnim->GetWheeledVehicleMovementComponent()->Wheels[(uint8)WheelLocation]->GetSteerAngle();
  }
  else
  {
    return VehicleAnim->GetWheelRotAngle((uint8)WheelLocation);
  }
}

void ACarlaWheeledVehicle::SetSimulatePhysics(bool enabled) {
  if(!GetCarlaMovementComponent<UDefaultMovementComponent>())
  {
    return;
  }

  UWheeledVehicleMovementComponent* Movement = GetVehicleMovement();
  if (Movement)
  {
    check(Movement != nullptr);

    if(bPhysicsEnabled == enabled)
      return;

    PhysicsCheckpoint.Reset();

    SetActorEnableCollision(true);
    auto RootComponent = Cast<UPrimitiveComponent>(GetRootComponent());
    RootComponent->SetSimulatePhysics(enabled);
    RootComponent->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);

    UVehicleAnimInstance *VehicleAnim = Cast<UVehicleAnimInstance>(GetMesh()->GetAnimInstance());
    check(VehicleAnim != nullptr)

    GetWorld()->GetPhysicsScene()->GetPxScene()->lockWrite();
    if (enabled)
    {
      Movement->RecreatePhysicsState();
      VehicleAnim->ResetWheelCustomRotations();
    }
    else
    {
      Movement->DestroyPhysicsState();
    }

    GetWorld()->GetPhysicsScene()->GetPxScene()->unlockWrite();

    bPhysicsEnabled = enabled;

    ResetConstraints();
  }

}

void ACarlaWheeledVehicle::ResetConstraints()
{
  for (int i = 0; i < ConstraintsComponents.Num(); i++)
  {
    OpenDoorPhys(EVehicleDoor(i));
  }
  for (int i = 0; i < ConstraintsComponents.Num(); i++)
  {
    CloseDoorPhys(EVehicleDoor(i));
  }
}

FVector ACarlaWheeledVehicle::GetVelocity() const
{
  return BaseMovementComponent->GetVelocity();
}

void ACarlaWheeledVehicle::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
  ShowDebugTelemetry(false);
  Super::EndPlay(EndPlayReason);
  RemoveReferenceToManager();
}

void ACarlaWheeledVehicle::OpenDoor(const EVehicleDoor DoorIdx) {
  if (int(DoorIdx) >= ConstraintsComponents.Num() && DoorIdx != EVehicleDoor::All) {
    UE_LOG(LogTemp, Warning, TEXT("This door is not configured for this car."));
    return;
  }

  if (DoorIdx == EVehicleDoor::All) {
    for (int i = 0; i < ConstraintsComponents.Num(); i++)
    {
      OpenDoorPhys(EVehicleDoor(i));
    }
    return;
  }

  OpenDoorPhys(DoorIdx);
}

void ACarlaWheeledVehicle::CloseDoor(const EVehicleDoor DoorIdx) {
  if (int(DoorIdx) >= ConstraintsComponents.Num() && DoorIdx != EVehicleDoor::All) {
    UE_LOG(LogTemp, Warning, TEXT("This door is not configured for this car."));
    return;
  }

  if (DoorIdx == EVehicleDoor::All) {
    for (int i = 0; i < ConstraintsComponents.Num(); i++)
    {
      CloseDoorPhys(EVehicleDoor(i));
    }
    return;
  }

  CloseDoorPhys(DoorIdx);
}

void ACarlaWheeledVehicle::OpenDoorPhys(const EVehicleDoor DoorIdx)
{
  PhysicsCheckpoint.Reset();
  UPhysicsConstraintComponent* Constraint = ConstraintsComponents[static_cast<int>(DoorIdx)];
  UPrimitiveComponent* DoorComponent = ConstraintDoor[Constraint];
  DoorComponent->DetachFromComponent(
      FDetachmentTransformRules(EDetachmentRule::KeepWorld, false));
  FTransform DoorInitialTransform =
      DoorComponentsTransform[DoorComponent] * GetActorTransform();
  DoorComponent->SetWorldTransform(DoorInitialTransform);
  DoorComponent->SetSimulatePhysics(true);
  DoorComponent->SetCollisionProfileName(TEXT("BlockAll"));
  float AngleLimit = Constraint->ConstraintInstance.GetAngularSwing1Limit();
  FRotator AngularRotationOffset = Constraint->ConstraintInstance.AngularRotationOffset;

  if (Constraint->ConstraintInstance.AngularRotationOffset.Yaw < 0.0f)
  {
    AngleLimit = -AngleLimit;
  }
  Constraint->SetAngularOrientationTarget(FRotator(0, AngleLimit, 0));
  Constraint->SetAngularDriveParams(DoorOpenStrength, 1.0, 0.0);

  Constraint->InitComponentConstraint();

  UPhysicsConstraintComponent** CollisionDisable =
      CollisionDisableConstraints.Find(DoorComponent);
  if (CollisionDisable)
  {
    (*CollisionDisable)->InitComponentConstraint();
  }
}

void ACarlaWheeledVehicle::CloseDoorPhys(const EVehicleDoor DoorIdx)
{
  PhysicsCheckpoint.Reset();
  UPhysicsConstraintComponent* Constraint = ConstraintsComponents[static_cast<int>(DoorIdx)];
  UPrimitiveComponent* DoorComponent = ConstraintDoor[Constraint];
  FTransform DoorInitialTransform =
      DoorComponentsTransform[DoorComponent] * GetActorTransform();
  DoorComponent->SetSimulatePhysics(false);
  DoorComponent->SetCollisionProfileName(TEXT("NoCollision"));
  DoorComponent->SetWorldTransform(DoorInitialTransform);
  DoorComponent->AttachToComponent(
      GetMesh(), FAttachmentTransformRules(EAttachmentRule::KeepWorld, true));
}

void ACarlaWheeledVehicle::ApplyRolloverBehavior()
{
  auto roll = GetVehicleTransform().Rotator().Roll;

  // The angular velocity reduction is applied in 4 stages, to improve its smoothness.
  // Case 4 starts the timer to set the rollover flag, so users are notified.
  switch (RolloverBehaviorTracker) {
    case 0: CheckRollover(roll, std::make_pair(130.0, 230.0));      break;
    case 1: CheckRollover(roll, std::make_pair(140.0, 220.0));      break;
    case 2: CheckRollover(roll, std::make_pair(150.0, 210.0));      break;
    case 3: CheckRollover(roll, std::make_pair(160.0, 200.0));      break;
    case 4:
      GetWorld()->GetTimerManager().SetTimer(TimerHandler, this, &ACarlaWheeledVehicle::SetRolloverFlag, RolloverFlagTime);
      RolloverBehaviorTracker += 1;
      break;
    case 5: break;
    default:
      RolloverBehaviorTracker = 5;
  }

  // In case the vehicle recovers, reset the rollover tracker
  if (RolloverBehaviorTracker > 0 && -30 < roll && roll < 30){
    RolloverBehaviorTracker = 0;
    FailureState = carla::rpc::VehicleFailureState::None;
  }
}

void ACarlaWheeledVehicle::CheckRollover(const float roll, const std::pair<float, float> threshold_roll){
  if (threshold_roll.first < roll && roll < threshold_roll.second){
    auto RootComponent = Cast<UPrimitiveComponent>(GetRootComponent());
    auto angular_velocity = RootComponent->GetPhysicsAngularVelocityInDegrees();
    RootComponent->SetPhysicsAngularVelocity((1 - RolloverBehaviorForce) * angular_velocity);
    RolloverBehaviorTracker += 1;
  }
}

void ACarlaWheeledVehicle::SetRolloverFlag(){
  // Make sure the vehicle hasn't recovered since the timer started
  if (RolloverBehaviorTracker >= 4) {
    FailureState = carla::rpc::VehicleFailureState::Rollover;
  }
}

carla::rpc::VehicleFailureState ACarlaWheeledVehicle::GetFailureState() const{
  return FailureState;
}

void ACarlaWheeledVehicle::AddReferenceToManager()
{
  const UObject* World = GetWorld();
  TArray<AActor*> ActorsInLevel;
  UGameplayStatics::GetAllActorsOfClass(World, AActor::StaticClass(), ActorsInLevel);
  for (AActor* Actor : ActorsInLevel)
  {
    AVegetationManager* Manager = Cast<AVegetationManager>(Actor);
    if (!IsValid(Manager))
      continue;
    Manager->AddVehicle(this);
    return;
  }
}

void ACarlaWheeledVehicle::RemoveReferenceToManager()
{
  const UObject* World = GetWorld();
  TArray<AActor*> ActorsInLevel;
  UGameplayStatics::GetAllActorsOfClass(World, AActor::StaticClass(), ActorsInLevel);
  for (AActor* Actor : ActorsInLevel)
  {
    AVegetationManager* Manager = Cast<AVegetationManager>(Actor);
    if (!IsValid(Manager))
      continue;
    Manager->RemoveVehicle(this);
    return;
  }
}

FRotator ACarlaWheeledVehicle::GetPhysicsConstraintAngle(
    UPhysicsConstraintComponent* Component)
{
  return Component->ConstraintInstance.AngularRotationOffset;
}

void ACarlaWheeledVehicle::SetPhysicsConstraintAngle(
    UPhysicsConstraintComponent* Component, const FRotator &NewAngle)
{
  Component->ConstraintInstance.AngularRotationOffset = NewAngle;
}
