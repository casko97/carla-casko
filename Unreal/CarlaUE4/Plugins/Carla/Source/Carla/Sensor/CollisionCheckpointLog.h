// This work is licensed under the terms of the MIT license.

#pragma once

#include "Carla/Sensor/Sensor.h"

// Experimental diagnostic log; it mirrors the sensor's de-duplicated hit
// events, not all PhysX contacts. Access must remain on the game thread.
struct FCollisionCheckpointLogEvent
{
  uint64 Frame;
  uint32 OtherActorId;
  FString OtherActorType;
  FVector NormalImpulse;
};

struct FCollisionCheckpointLog
{
  std::vector<FCollisionCheckpointLogEvent> Events;
  bool bOverflow = false;
};
