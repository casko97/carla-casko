// Copyright (c) 2020 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "Carla.h"
#include "Carla/Game/CarlaStatics.h"
#include "TrafficLightGroup.h"
#include "TrafficLightBase.h"
#include "Carla/Game/CarlaEpisode.h"
#include <carla/rpc/PortableTrafficLightsCheckpoint.h>

struct FTrafficLightGroupCheckpoint
{
  int CurrentController;
  bool bIsFrozen;
  TArray<TWeakObjectPtr<UTrafficLightController>> Controllers;
  TArray<FTrafficLightControllerCheckpoint> ControllerStates;
};

// Bind actor identities to OpenDRIVE signal IDs instead of episode-local IDs.
static bool PortableActorSignId(UCarlaEpisode *Episode, uint32 Id, std::string &Out)
{
  auto *CarlaActor = Episode ? Episode->FindCarlaActor(Id) : nullptr;
  auto *Actor = CarlaActor ? Cast<ATrafficLightBase>(CarlaActor->GetActor()) : nullptr;
  auto *Component = Actor ? Actor->GetTrafficLightComponent() : nullptr;
  if (!Component || Component->GetSignId().IsEmpty()) return false;
  Out = TCHAR_TO_UTF8(*Component->GetSignId());
  return true;
}

bool ATrafficLightGroup::ExportPortableCheckpoint(carla::rpc::PortableLightGroup &Out) const
{
  auto *Episode = UCarlaStatics::GetCurrentEpisode(GetWorld());
  if (!Episode || !Controllers.IsValidIndex(CurrentController)) return false;
  Out.junction_id = JunctionId;
  Out.current_controller = CurrentController;
  Out.frozen = bIsFrozen;
  for (auto *Controller : Controllers)
  {
    FTrafficLightControllerCheckpoint State;
    if (!IsValid(Controller) || !Controller->CaptureCheckpoint(State)) return false;
    carla::rpc::PortableLightController Data;
    Data.id = TCHAR_TO_UTF8(*Controller->GetControllerId());
    Data.current_state = State.CurrentState;
    Data.sequence = State.Sequence;
    Data.elapsed = State.ElapsedTime;
    Data.current_light_state = static_cast<uint8>(State.CurrentLightState);
    for (const auto &Stage : State.LightStates)
      Data.stages.push_back({Stage.Time, static_cast<uint8>(Stage.State)});
    for (int32 I = 0; I < State.Components.Num(); ++I)
    {
      auto *Component = State.Components[I].Get();
      if (!Component || Component->GetSignId().IsEmpty()) return false;
      Data.component_ids.emplace_back(TCHAR_TO_UTF8(*Component->GetSignId()));
      Data.component_states.push_back(static_cast<uint8>(State.ComponentStates[I]));
    }
    for (int32 I = 0; I < State.ActorIds.Num(); ++I)
    {
      std::string Id;
      if (!PortableActorSignId(Episode, State.ActorIds[I], Id)) return false;
      Data.actor_ids.push_back(std::move(Id));
      Data.actor_states.push_back(static_cast<uint8>(State.ActorStates[I]));
    }
    Out.controllers.push_back(std::move(Data));
  }
  return true;
}

bool ATrafficLightGroup::PreparePortableCheckpoint(const carla::rpc::PortableLightGroup &In)
{
  auto *Episode = UCarlaStatics::GetCurrentEpisode(GetWorld());
  if (!Episode || JunctionId != In.junction_id || Controllers.Num() != static_cast<int32>(In.controllers.size()) ||
      !Controllers.IsValidIndex(In.current_controller)) return false;
  auto Snapshot = MakeShared<FTrafficLightGroupCheckpoint>();
  Snapshot->CurrentController = In.current_controller;
  Snapshot->bIsFrozen = In.frozen;
  for (int32 I = 0; I < Controllers.Num(); ++I)
  {
    auto *Controller = Controllers[I];
    const auto &Data = In.controllers[I];
    FTrafficLightControllerCheckpoint State;
    if (!IsValid(Controller) || Data.id != TCHAR_TO_UTF8(*Controller->GetControllerId()) ||
        !Controller->CaptureCheckpoint(State) || State.Components.Num() != static_cast<int32>(Data.component_ids.size()) ||
        State.ActorIds.Num() != static_cast<int32>(Data.actor_ids.size())) return false;
    for (int32 J = 0; J < State.Components.Num(); ++J)
    {
      auto *Component = State.Components[J].Get();
      if (!Component || Data.component_ids[J] != TCHAR_TO_UTF8(*Component->GetSignId())) return false;
      State.ComponentStates[J] = static_cast<ETrafficLightState>(Data.component_states[J]);
    }
    for (int32 J = 0; J < State.ActorIds.Num(); ++J)
    {
      std::string Id;
      if (!PortableActorSignId(Episode, State.ActorIds[J], Id) || Data.actor_ids[J] != Id) return false;
      State.ActorStates[J] = static_cast<ETrafficLightState>(Data.actor_states[J]);
    }
    State.CurrentState = Data.current_state;
    State.Sequence = Data.sequence;
    State.ElapsedTime = Data.elapsed;
    State.CurrentLightState = static_cast<ETrafficLightState>(Data.current_light_state);
    State.LightStates.Empty();
    for (const auto &Stage : Data.stages)
      State.LightStates.Add({Stage.time, static_cast<ETrafficLightState>(Stage.state)});
    if (!Controller->ValidateCheckpoint(State)) return false;
    Snapshot->Controllers.Add(Controller);
    Snapshot->ControllerStates.Add(MoveTemp(State));
  }
  TrafficLightCheckpoint = MoveTemp(Snapshot);
  return true;
}

bool ATrafficLightGroup::SaveCheckpoint()
{
  if (!Controllers.IsValidIndex(CurrentController)) return false;
  auto Snapshot = MakeShared<FTrafficLightGroupCheckpoint>();
  Snapshot->CurrentController = CurrentController;
  Snapshot->bIsFrozen = bIsFrozen;
  for (auto *Controller : Controllers)
  {
    if (!IsValid(Controller)) return false;
    FTrafficLightControllerCheckpoint State;
    if (!Controller->CaptureCheckpoint(State)) return false;
    Snapshot->Controllers.Add(Controller);
    Snapshot->ControllerStates.Add(MoveTemp(State));
  }
  TrafficLightCheckpoint = MoveTemp(Snapshot);
  return true;
}

bool ATrafficLightGroup::ValidateCheckpoint() const
{
  if (!TrafficLightCheckpoint || !Controllers.IsValidIndex(TrafficLightCheckpoint->CurrentController) ||
      Controllers.Num() != TrafficLightCheckpoint->Controllers.Num() ||
      Controllers.Num() != TrafficLightCheckpoint->ControllerStates.Num()) return false;
  for (int32 I = 0; I < Controllers.Num(); ++I)
    if (!TrafficLightCheckpoint->Controllers[I].IsValid() ||
        Controllers[I] != TrafficLightCheckpoint->Controllers[I].Get() ||
        !Controllers[I]->ValidateCheckpoint(TrafficLightCheckpoint->ControllerStates[I])) return false;
  return true;
}

void ATrafficLightGroup::RestoreCheckpoint()
{
  check(ValidateCheckpoint());
  CurrentController = TrafficLightCheckpoint->CurrentController;
  bIsFrozen = TrafficLightCheckpoint->bIsFrozen;
  for (int32 I = 0; I < Controllers.Num(); ++I)
    Controllers[I]->RestoreCheckpoint(TrafficLightCheckpoint->ControllerStates[I]);
}


// Sets default values
ATrafficLightGroup::ATrafficLightGroup()
{
  // Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
  PrimaryActorTick.bCanEverTick = true;
  SceneComponent = CreateDefaultSubobject<USceneComponent>(TEXT("RootComponent"));
  RootComponent = SceneComponent;
}

void ATrafficLightGroup::SetFrozenGroup(bool InFreeze)
{
  bIsFrozen = InFreeze;
}

bool ATrafficLightGroup::IsFrozen() const
{
  return bIsFrozen;
}

void ATrafficLightGroup::ResetGroup()
{
  for(auto * Controller : Controllers)
  {
    Controller->ResetState();
  }
  CurrentController = 0;
  UTrafficLightController* controller = Controllers[CurrentController];
  controller->StartCycle();
}

// Called every frame
void ATrafficLightGroup::Tick(float DeltaTime)
{
  TRACE_CPUPROFILER_EVENT_SCOPE(ATrafficLightGroup::Tick);
  Super::Tick(DeltaTime);

  // Do not update if the replayer is replaying
  auto* Episode = UCarlaStatics::GetCurrentEpisode(GetWorld());
  if (Episode)
  {
    auto* Replayer = Episode->GetReplayer();
    if (Replayer)
    {
      if(Replayer->IsEnabled())
      {
        return;
      }
    }
  }

  if (bIsFrozen)
  {
    return;
  }

  UTrafficLightController* controller = Controllers[CurrentController];
  if (controller->AdvanceTimeAndCycleFinished(DeltaTime))
  {
    NextController();
  }
}

void ATrafficLightGroup::NextController()
{
  CurrentController = (CurrentController + 1) % Controllers.Num();
  UTrafficLightController* controller = Controllers[CurrentController];
  controller->StartCycle();
}

int ATrafficLightGroup::GetJunctionId() const
{
  return JunctionId;
}

void ATrafficLightGroup::AddController(UTrafficLightController* Controller)
{
  Controllers.Add(Controller);
  Controller->SetGroup(this);
}
