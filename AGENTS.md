# Repository development conventions

- Keep user-facing architecture, setup and extension documentation in Chinese.
- Keep `robot_core` independent of ROS, BehaviorTree.CPP, model SDKs and vendor SDKs.
- A skill owns goal semantics and verification. A component supplies a technical function.
- Component requirements belong to each skill implementation, not a specific model name.
- Never treat action acceptance or cancellation acceptance as verified physical completion.
- Preserve resource ownership while stop is unconfirmed; no blind retry after a timeout.
- All demo backends are mock-only. Do not silently add hardware commands or credentials.
- Run `bash scripts/test_core.sh` after core changes. For ROS changes run the Jazzy CI build
  and `python3 scripts/test_ros.py`; report when ROS validation cannot run locally.
- Keep the main executor single-threaded until ownership and synchronization rules are redesigned.
- Do not add a specific model integration, plugin dependency or broad new abstraction without
  a concrete requirement. Prefer a small working example plus explicit extension documentation.
- The repository owner has not selected an open-source license. Preserve that decision.
