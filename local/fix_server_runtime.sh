#!/usr/bin/env bash
# Adapt generated server archives to UE4's bundled pre-C23 glibc sysroot on x86-64.
# Run after rebuilding server dependencies and before building CarlaUE4Editor.
set -euo pipefail
root="$(cd "$(dirname "$0")/.." && pwd)"
libdir="$root/Unreal/CarlaUE4/Plugins/Carla/CarlaDependencies/lib"
for lib in libsqlite3.a libxerces-c.a libproj.a libosm2odr.a; do
  test -f "$libdir/$lib"
  objcopy --redefine-sym __isoc23_sscanf=__isoc99_sscanf \
    --redefine-sym __isoc23_strtol=strtol --redefine-sym __isoc23_strtoul=strtoul \
    --redefine-sym __isoc23_strtoll=strtoll --redefine-sym __isoc23_strtoull=strtoull \
    --redefine-sym fcntl64=fcntl "$libdir/$lib"
done
