#!/usr/bin/env bash
set -euo pipefail
root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../deterministic-cpt" && pwd -P)
engine=${UE4_ROOT:-/home/casko/phd-code/carla-casko-engine}
case "${1:-}" in
  check)
    python3 - "$root" <<'PY'
import ast
from pathlib import Path
import sys
root = Path(sys.argv[1])
files = sorted((root / 'PythonAPI/examples').glob('*checkpoint*.py'))
for path in files:
    ast.parse(path.read_bytes(), filename=str(path))
print(f'Parsed {len(files)} checkpoint Python scripts without creating bytecode.')
for name in ('client', 'server'):
    cache = root / f'Build/libcarla-{name}-build.release/CMakeCache.txt'
    if not cache.exists():
        print(f'{name}: no CMake cache')
        continue
    entries = dict(line.split('=', 1) for line in cache.read_text().splitlines()
                   if line.startswith(('CMAKE_HOME_DIRECTORY:INTERNAL=', 'CMAKE_CACHEFILE_DIR:INTERNAL=')))
    source = entries.get('CMAKE_HOME_DIRECTORY:INTERNAL')
    build = entries.get('CMAKE_CACHEFILE_DIR:INTERNAL')
    expected_build = str(cache.parent)
    if source != str(root) or build != expected_build:
        print(f'{name}: UNSAFE copied cache (source={source}, build={build}); do not run its build')
    else:
        print(f'{name}: cache paths match this worktree')
PY
    ;;
  editor-plan)
    make -n -C "$root/Unreal/CarlaUE4" CarlaUE4Editor
    ;;
  editor)
    [[ -x "$engine/Engine/Build/BatchFiles/Linux/Build.sh" ]] || { echo "Engine build tool not found: $engine" >&2; exit 1; }
    makefile="$root/Unreal/CarlaUE4/Makefile"
    grep -Fxq "UNREALROOTPATH = $engine" "$makefile" || { echo 'Makefile uses a different engine; regenerate it before building.' >&2; exit 1; }
    grep -Fxq "GAMEPROJECTFILE =$root/Unreal/CarlaUE4/CarlaUE4.uproject" "$makefile" || { echo 'Makefile uses a different project; regenerate it before building.' >&2; exit 1; }
    echo 'Building the editor target incrementally; do not run concurrently with another build using this engine.'
    exec make -C "$root/Unreal/CarlaUE4" CarlaUE4Editor
    ;;
  *)
    echo "Usage: bash $0 {check|editor-plan|editor}" >&2
    exit 2
    ;;
esac
