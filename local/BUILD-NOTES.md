# CARLA 0.9.15 local build

This checkout is based on the 0.9.15 tag, with local compatibility changes for Ubuntu 24.04. The engine is at `../carla-casko-engine`, on the CARLA Unreal 4.26 fork. The engine's dependency manifest was replaced with Epic's refreshed 4.26 manifest; its shared entries have the same hashes as the fork's original manifest, and the refreshed version omits an unavailable license PDF.

The Python 3.8 client is installed in `.venv`; it reports `0.9.15-dirty` because the source has local fixes. The source asset version is `20231108_c5101a5`. Build changes include updated dependency URLs, static/PIC dependency builds, Python header lookup and a missing CARLA header include. The generated server libraries need compatibility symbol remapping because Ubuntu 24.04 headers otherwise target C23 symbols absent from Unreal's older sysroot. `local/fix_server_runtime.sh` repeats that x86-64-specific adaptation; it restores legacy pre-C23 parsing behavior and maps `fcntl64` to `fcntl` (the same ABI on x86-64). This local workaround needs review for distribution or other architectures. It must be rerun after rebuilding those archives.

To rebuild the project after modifying source, from this directory:

```bash
export UE4_ROOT="$HOME/phd-code/carla-casko-engine"
export PATH="$PWD/.venv/bin:$PATH"
export LDFLAGS=-fuse-ld=/usr/bin/ld
make LibCarla.server.release ARGS='--python-version=3.8'
Util/BuildTools/BuildOSM2ODR.sh --build
local/fix_server_runtime.sh
Util/BuildTools/BuildUE4Plugins.sh --build
Util/BuildTools/BuildCarlaUE4.sh --build
```

To rebuild the Python client after changing CARLA's client code, from this directory:

```bash
export UE4_ROOT="$HOME/phd-code/carla-casko-engine"
export PATH="$PWD/.venv/bin:$PATH"
export LDFLAGS=-fuse-ld=/usr/bin/ld
make LibCarla.client.release ARGS='--python-version=3.8'
make osm2odr
unset LDFLAGS
export LDSHARED="$UE4_ROOT/Engine/Extras/ThirdPartyNotUE/SDKs/HostLinux/Linux_x64/v17_clang-10.0.1-centos7/x86_64-unknown-linux-gnu/bin/clang++ -shared -fuse-ld=/usr/bin/ld"
Util/BuildTools/BuildPythonAPI.sh --python-version=3.8
```

Install the resulting wheel into `.venv` before testing. The extension link is split out because the host linker flag used for building dependencies is not accepted by the Python interpreter's default GCC linker command.

Start a server off-screen on a free port:

```bash
"$UE4_ROOT/Engine/Binaries/Linux/UE4Editor" "$PWD/Unreal/CarlaUE4/CarlaUE4.uproject" /Game/Carla/Maps/Town01 -game -RenderOffScreen -vulkan -nosound -carla-rpc-port=2200 -quality-level=Low -unattended
```

Once the server is ready, run `.venv/bin/python local/smoke_test.py --port 2200`. First startup compiles shaders and textures and can take a long time.

## Packaged-launcher compatibility

`CarlaUE4.sh` at the repository root is an executable adapter for tools that
expect the packaged CARLA launcher, including Bench2Drive. It resolves this
checkout's sibling engine (`../carla-casko-engine`) by default and forwards all
arguments unchanged to `UE4Editor`. Set `UE4_ROOT` to override that location.

For Bench2DriveZoo, point its configuration at this source checkout:

```bash
export CARLA_ROOT="$HOME/phd-code/carla-casko"
```

The evaluator's `-RenderOffScreen`, `-nosound`, RPC-port, and GPU flags are
forwarded by the wrapper. It starts on Town01 only for initial server startup;
the evaluator loads each route's requested town before running the scenario.
