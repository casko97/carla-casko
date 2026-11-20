#!/usr/bin/env bash
# Source-build compatibility launcher for CARLA clients that expect the
# packaged CarlaUE4.sh entry point (for example, Bench2Drive's evaluator).
set -euo pipefail

carla_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ue4_root="${UE4_ROOT:-${carla_root}/../carla-casko-engine}"
editor="${ue4_root}/Engine/Binaries/Linux/UE4Editor"
project="${carla_root}/Unreal/CarlaUE4/CarlaUE4.uproject"

if [[ ! -x "${editor}" ]]; then
  printf 'CARLA source launcher: UE4Editor is not executable: %s\n' "${editor}" >&2
  printf 'Set UE4_ROOT to the CARLA Unreal Engine checkout if it is elsewhere.\n' >&2
  exit 1
fi
if [[ ! -f "${project}" ]]; then
  printf 'CARLA source launcher: project file is missing: %s\n' "${project}" >&2
  exit 1
fi

# Start a small standard map while the client establishes its connection.
# Benchmark evaluators subsequently load the route's requested town themselves.
exec "${editor}" "${project}" /Game/Carla/Maps/Town01 -game "$@"
