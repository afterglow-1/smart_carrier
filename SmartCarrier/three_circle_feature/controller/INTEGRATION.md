# SmartCarrier integration points

This directory is intentionally separate from `SmartCarrier/src` and
`SmartCarrier/lib`. The original route program has not been modified.

## Objects to add to the feature main

```cpp
#include "ThreeCircleVision.h"
#include "VisualTaskCoordinator.h"
#include "ArmTaskPlan.h"

HardwareSerial Serial_Maix(PE7, PE8);
ThreeCircleFeature::ThreeCircleVision threeCircleVision(Serial_Maix);
ThreeCircleFeature::VisualTaskCoordinator visualTask(threeCircleVision);
ThreeCircleFeature::ArmTaskPlan armTaskPlan;
```

Call `threeCircleVision.begin()` during `setup()`. Call `threeCircleVision.poll()`
or `visualTask.update()` on every pass through `loop()` so a split UART packet
does not get lost.

## Route state changes

Add these states to the copied feature version of `ProgramState`:

```cpp
VISION_ACQUIRE,
ARM_TASK_RUNNING,
ARM_TASK_FAILED,
```

In `handleArrival()`, replace the normal checkpoint dwell for these four
actions with `visualTask.beginAcquire()` and `ProgramState::VISION_ACQUIRE`:

- `PROCESS_AND_LOAD_BATCH_1`: rough processing, batch 1
- `STORE_BATCH_1`: temporary storage, batch 1
- `PROCESS_AND_LOAD_BATCH_2`: rough processing, batch 2
- `STORE_BATCH_2`: temporary storage, batch 2

These route entries already occur after the corresponding chassis turn, so
the arm/camera faces the work area before acquisition starts.

In the `VISION_ACQUIRE` loop branch:

1. Keep the wheel drivers disabled and call `visualTask.update()`.
2. On `Ready`, call `buildArmTaskPlan(zone, batch, result, armTaskPlan)`.
3. Start `ARM_TASK_RUNNING` only when the plan builds successfully.
4. On timeout, bad quality, stale data or an uncalibrated pose, enter
   `ARM_TASK_FAILED`; do not continue the route or move any arm axis.

The seven plan actions are: safe pose, pick the block from the tray, aim at
numbers 1, 2 and 3 in order, return the block to the tray, then safe pose.
Advancing between actions must use completion feedback from M5, ID6, ID7 and
servo ID4. Do not use fixed blocking delays in the route loop.

## Start-button change requested by the task

The existing main uses two clicks: first click clamps, second click starts the
route. For a literal one-click start, change `updateInitialGripperClamp()` so
that completion calls `startRouteFromBeginning()` instead of entering
`WAITING_TO_START`. This must be confirmed against the intended preload and
operator safety procedure before implementation.
