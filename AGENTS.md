# AGENTS.md

## Fast start (CI parity)
- Initialize submodules before configuring: `git submodule update --init --recursive` (`third_party/hikcamera` is required by root `CMakeLists.txt`).
- CI toolchain is Ubuntu 24.04 with `gcc-14`/`g++-14`; this project uses C++23.
- `scripts/install_dependencies.sh` is the CI dependency source of truth (installs OpenVINO 2025.2 and required apt packages; uses `sudo`).
- Use CI command order when validating locally:
  1. `bash tests/download_assets.sh`
  2. `cmake -S . -B build -DCMAKE_BUILD_TYPE=Release`
  3. `cmake --build build --parallel`
  4. `ctest --test-dir build --output-on-failure --no-tests=error`
- `Release` adds `-O3 -march=native`; avoid using default Release artifacts as portable binaries across different CPUs.

## Build and test shortcuts
- Build only replay tool: `cmake --build build --target pingpong_tracker_predictor_replay --parallel`.
- Run one ctest by regex: `ctest --test-dir build -R "^ReplayEvaluation\\." --output-on-failure`.
- Run one gtest binary directly: `./build/tests/replay_evaluation_test --gtest_filter=ReplayEvaluation.*`.
- `test_serializable` must run with working directory `build/tests` (it loads `serializable.yaml` via relative path).
- `model_test` data paths:
  - image: `${TEST_ASSETS_ROOT:-/tmp/pingpong_tracker}/pingpong.png`
  - model: `${TEST_MODELS_ROOT:-<repo>/models}/yolov8.onnx`
  Missing files are skipped by gtest, so run `tests/download_assets.sh` for full coverage.

## Entry points and execution flow
- Executables:
  - `src/runtime.cpp` -> `pingpong_tracker_runtime` (online loop: capturer -> identifier -> optional visualization).
  - `src/predictor_replay.cpp` -> `pingpong_tracker_predictor_replay` (offline CSV replay RMSE evaluator).
- Runtime config is loaded from `<repo>/config/config.yaml` via compile-time `PINGPONG_TRACKER_SOURCE`, not process cwd.
- Default `config/config.yaml` uses `capturer.source: local_video` with a hardcoded video path; adjust before running on a new machine.
- `capturer.source` currently works for `local_video` and `hikcamera`; `images` is stubbed and not wired.
- Predictor replay CLI:
  `./build/pingpong_tracker_predictor_replay <csv> [--config <yaml>] [--horizon-ms N] [--step-ms N] [--warmup N]`
  CSV must have strictly increasing timestamps and columns for time + `x,y,z` (time aliases are supported).

## Codebase map
- `src/module/**`: core implementations (capturer adapters, OpenVINO identifier, predictor UKF/IMM, shared utilities).
- `src/kernel/**`: runtime-facing wrappers/orchestration over module components.
- `tests/*.cpp`: all gtest suites, registered via `gtest_discover_tests` in `tests/CMakeLists.txt`.

## Style and analysis configs
- `.clang-format`: Google base, 4-space indent, 100-column limit.
- `.clang-tidy`: broad checks (`bugprone`, `google`, `modernize`, `performance`, `concurrency`, `cppcoreguidelines`) with selected groups treated as errors.
- Naming rules in `.clang-tidy`: functions/variables `lower_case`, private/protected members end with `_`, constants use `k` prefix.
