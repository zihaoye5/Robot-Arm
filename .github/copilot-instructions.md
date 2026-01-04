# Copilot instructions for Zero Robotic Arm

Purpose: help AI coding agents become productive quickly in this repository.

- **Big picture:** This repo contains a SolidWorks model set, embedded firmware (STM32 + FreeRTOS), Simulink kinematics, and Deep-RL training code. Key boundaries:
  - Embedded firmware: [2. Software/robot](2.%20Software/robot)
  - Kinematics & simulation: [Simulink](Simulink)
  - Reinforcement learning and MuJoCo models: [5. Deep_LR](5.%20Deep_LR)

- **Architecture notes:**
  - Embedded code is C with FreeRTOS. Main control loop and task creation are in [2. Software/robot/Core/Src/robot.c](2.%20Software/robot/Core/Src/robot.c).
  - Inverse kinematics computations live in [2. Software/robot/Core/Src/robot_kinematics.c](2.%20Software/robot/Core/Src/robot_kinematics.c).
  - Command parsing / dispatch lives in [2. Software/robot/Core/Src/robot_cmd.c](2.%20Software/robot/Core/Src/robot_cmd.c) (see `robot_uart1_cmd_table`).
  - Communication: CAN for motor comms, UART for console/commands, MQTT hook points exist (see USART and `robot_mqtt_handle`).

- **Developer workflows (concrete commands & files):**
  - Embedded build & debug: use STM32CubeMX / STM32CubeIDE (project files and linker scripts in `2. Software/robot`), open the CubeIDE workspace to flash and debug.
  - Local editing: code is written to be edited in VS Code; treat CubeMX-generated files as source-of-truth for peripheral init.
  - Simulink: run [Simulink/robot_run.m](Simulink/robot_run.m#L1) to launch the URDF-based sim model for kinematics verification.
  - Deep RL training: run `python train_robot_arm.py` in [5. Deep_LR](5.%20Deep_LR) — test mode example in README: `python train_robot_arm.py --test --model-path ./logs/best_model/best_model.zip --normalize-path ./logs/best_model/vec_normalize.pkl --episodes 50`.

- **Conventions & patterns to respect:**
  - Path casing and numbering: many top-level folders are prefixed (e.g., `2. Software`, `5. Deep_LR`). Preserve these paths exactly when creating links or editing files.
  - Task names appear in code and logs (e.g., `robot_control_task`, `robot_cmd_service`) — use these names when searching logs or creating tests that check RTOS behavior.
  - Command queue and message sizes are defined in [2. Software/robot/Core/Inc/robot.h](2.%20Software/robot/Core/Inc/robot.h) (e.g., `ROBOT_CMD_LENGTH`, `ROBOT_CMD_MAX_NUM`) — follow these limits in any code that formats or injects commands.
  - Use existing macros like `LOG()` for consistency when adding debug prints.

- **Integration points to watch:**
  - `robot_cmd_send_from_isr` and `xQueueSendToBackFromISR` — ISR → task handoff patterns appear in `2. Software/robot/Core/Src/robot.c` and `Core/Src/usart.c`.
  - MQTT hooks: `robot_mqtt_handle` is a thin handler — if adding network features, keep message format compatibility with `robot_uart1_cmd_table`.
  - Kinematics functions expose `robot_kinematics_cal_T(...)` and a global `g_robot_kinematics` result table — tests should validate both numeric outputs and expected branch choices.

- **Examples agents should use when changing behavior:**
  - To add a new serial command: follow the `robot_uart1_cmd_table` pattern in [2. Software/robot/Core/Src/robot_cmd.c](2.%20Software/robot/Core/Src/robot_cmd.c#L137).
  - To adjust reset/zero behavior: inspect the `hard_reset` / `soft_reset` handling and initial position setup in [2. Software/robot/Core/Src/robot.c](2.%20Software/robot/Core/Src/robot.c#L250-L340).
  - To validate inverse kinematics changes: run `Simulink/robot_run.m` and compare outputs with `robot_kinematics_cal_T` in [robot_kinematics.c](2.%20Software/robot/Core/Src/robot_kinematics.c#L481).

- **What not to change without testing on hardware/sim:**
  - CAN timing and `ROBOT_CAN_DELAY`/`ROBOT_CAN_TIMEOUT` constants — these affect motor comms and safety timeouts.
  - Limits and queue sizes in `robot.h` — changing sizes affects RTOS stability and memory.

- **Quick checklist for PRs from AI agents:**
  - Reference the exact path of edited files (use the numbered folder names).
  - Run Simulink script(s) for kinematics-sensitive changes or include unit test vectors when simulation cannot be used.
  - For embedded changes, run a static build (CubeIDE) or linting; document required CubeMX changes.

If anything here is unclear or you want more examples (e.g., a short PR template or concrete test vectors for kinematics), tell me which area to expand.
