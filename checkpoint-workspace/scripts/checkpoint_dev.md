# Incremental checkpoint development

Do not perform package builds for this work. Use the incremental editor target
and existing local assets for development experiments. Consider fixing a
simulator property as a complement to capturing it when representative tests
show that the resulting simulation quality remains acceptable; record the
setting, compatibility requirements, and observed effect.

The experimental source is in `deterministic-cpt`. Its editor target is separate
from the main checkout's Shipping package. Both use the engine at
`/home/casko/phd-code/carla-casko-engine`, including its derived-data cache.
Preserve the cache and the worktree's `Intermediate` and `Binaries` directories.

From the main checkout:

```bash
bash scripts/checkpoint_dev.sh check
bash scripts/checkpoint_dev.sh editor-plan
bash scripts/checkpoint_dev.sh editor
```

`check` parses the Python checkpoint harnesses and inspects CMake cache paths;
it does not run the simulator. For Python-only harness changes, no build is
needed: use the existing matching client and server. `editor-plan` prints the
generated editor build command. `editor` asks Unreal Build Tool to update the
checkpoint worktree's editor target without cleaning or cooking content. It
uses the existing engine and intermediates; Unreal decides what must rebuild.
Run it only after other builds using the same engine have finished, and with
permission to write the engine directory.

The current `Build/libcarla-{client,server}-build.release` directories were
copied from the main checkout: their CMake caches point there. Do not run
`make PythonAPI`, `make CarlaUE4Editor`, or `ninja` against those caches in this
worktree until separate worktree-local CMake build and install directories have
been configured. In particular, the editor target alone does not rebuild the
LibCarla server static library or the Python extension. Changes to LibCarla
or its Python bindings need matching client/server library and Python API
builds; the existing wheel may be stale. A safe independent client/server build
setup is still required before those changes can be tested.

Full packaging and cooking are only needed for a standalone distribution or
changed cooked assets. The main checkout's running package is not a checkpoint
build. `-nullrhi` may be useful for physics-only experiments, but cannot test
camera output. Do not infer simulator determinism from successful compilation.
