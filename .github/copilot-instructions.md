## Quick orientation

This repo implements a high-performance C++ port of an RL trainer for PX4
(REINFORCE + LibTorch) and includes helper scripts to install, build and run.
The canonical working folder for development is `RL_with_cpp/`.

Key entry points:
- `RL_with_cpp/install_and_setup.sh` — one-command installer (recommended).
- `RL_with_cpp/setup.sh` / `setup_env.sh` — manual dependency setup and env
  script creation.
- `RL_with_cpp/build.sh` — compile after code changes.
- `RL_with_cpp/run_training.sh` — convenience runner that picks RMW, sources
  environments and launches `ros2 run rl_with_cpp train_rl`.

Architecture overview (what to read first):
- `include/` — API surface: `px4_node.hpp`, `px4_accel_env.hpp`, `policy_network.hpp`.
- `src/` — implementations: `px4_node.cpp` (ROS2 interface), `px4_accel_env.cpp`
  (environment + rewards), `policy_network.cpp`, `train_rl.cpp` (main loop).
- `DOCUMENTATION_INDEX.md`, `QUICKSTART.md`, and `README.md` in `RL_with_cpp/`
  contain tested setup and run commands — prefer them over ad-hoc changes.

Important developer conventions and patterns
- Reward logic and all 13 reward terms live in `px4_accel_env.cpp` — change here
  to alter training semantics. The code aims to be a literal port of the Python
  version; keep hyperparameters aligned with `train_rl.cpp` when modifying.
- Policy network shape and learnable log-std live in `policy_network.*`.
- Checkpointing and filenames: `best_policy.pt`, `final_policy.pt`,
  `policy_scripted.pt`, `checkpoint_ep*.pt` — keep names consistent for tooling.
- JSON config & artifacts: `policy_config.json` is used to capture model shape.

Build / run quick references (copyable)
1) Automated install & train (recommended):
   cd RL_with_cpp
   ./install_and_setup.sh
   ./run_training.sh

2) Manual build (colcon) after edits:
   cd RL_with_cpp
   source setup_env.sh
   cd ../..  # workspace root
   colcon build --packages-select rl_with_cpp
   source install/setup.bash

Runtime & integration notes for agents
- `run_training.sh` chooses RMW implementation: prefers CycloneDDS, falls back to
  Fast DDS and (when used) points `FASTDDS_DEFAULT_PROFILES_FILE` at
  `fastdds_profile.xml`. Agents must respect `RMW_IMPLEMENTATION` and source
  ROS2 before interacting with topics.
- Trainer waits for PX4 topics — if training stalls, check `/fmu/*` topics
  (see README troubleshooting). MicroXRCEAgent / PX4 SITL must be running.
- Optional runtime flags: `--isaac-reset` is supported by `run_training.sh` via
  an environment flag (`ISAAC_SIM_RESET`) for integration with Isaac Sim.

What to avoid
- Do not change reward weights across files without updating `IMPLEMENTATION_SUMMARY.md`.
- Avoid modifying `run_training.sh` RMW logic unless you understand DDS implications
  (topics, QoS, Fast DDS payload realloc workarounds are documented in the script).

Common troubleshooting: Fast DDS payload size errors
- If you see "Change payload size... is larger than history payload size... and cannot
  be resized", the Fast DDS profile isn't loaded. Export system-wide before starting
  ANY ROS2/PX4 process:
  ```
  export FASTDDS_DEFAULT_PROFILES_FILE="$HOME/PegasusSimulator_51/examples/RL_test/ACRL-RL/RL_with_cpp/fastdds_profile.xml"
  ```
- Or add to `~/.bashrc` and restart all terminals. The profile enables
  `PREALLOCATED_WITH_REALLOC` to allow dynamic message resizing.

Small tasks an AI agent can safely do
- Update minor README sections, add missing examples, or fix typos in
  `QUICKSTART.md` and `DOCUMENTATION_INDEX.md`.
- Add unit-style smoke tests verifying that `train_rl` binary starts and
  that expected files (e.g. `policy_config.json`) are produced by setup scripts.

Where to ask follow-ups
- If unsure about a build failure, reference `RL_with_cpp/README.md` and
  `GETTING_STARTED.txt` first; otherwise ask the repo maintainer for ROS2/PX4
  environment details (ROS2 version and workspace layout).

If anything here is unclear or you want more detail for a specific task (e.g.
writing tests, refactoring the reward function, or adding CI), tell me which
area and I'll expand or produce PR-ready edits.
