# Three numbered-circle feature

This folder contains the first, isolated implementation stage. No original
SmartCarrier or MaixCam source file is modified.

## Implemented

- MaixCam detection of up to three complete or partial concentric-ring targets.
- Lightweight 5 x 7 digit feature matching for numbers 1, 2 and 3, with a
  frame-wide unique-number assignment.
- Board geometry at 0, 300 and 580 mm from the supplied drawing.
- Two numbered circles can infer the third centre, including a centre outside
  the camera frame.
- A weak third fragment is rejected in favour of the two-circle prediction; a
  reliable, geometrically consistent fragment is fused with the prediction.
- Fixed 30-byte UART packet with signed coordinates, result source, confidence,
  CRC16 and freshness sequence.
- STM32 parser, acquisition quality gate, four-station calibration table and a
  seven-step arm task plan.

## Still required before arm integration

1. Confirm that the drawing means centre positions `0/300/580 mm`. If `300`
   is a circle diameter rather than the first centre spacing, provide the full
   centre-to-centre dimensions.
2. Provide the outer physical circle diameter. It lets vision reject a wrong
   pair when only two circles are visible.
3. Capture original `320 x 240` MaixCam frames at the final chassis pose for
   rough processing and temporary storage, for both batches. Include normal,
   two-circle and severe-occlusion cases; 20-30 frames per pose is useful.
4. Provide the final camera mounting direction and whether image X/Y increase
   in the same direction at all four work poses.
5. Provide M5 base pulses, ID6 boom position and ID7 lift position for the safe
   pose, tray pickup/return, and numbers 1/2/3 at all four work poses.
6. Provide the legal ranges, home/reference procedure and completion feedback
   API for ID6 and ID7. Current `SmartCarrier/src/main.cpp` only drives M5 and
   servo ID4; the ID6/ID7 code exists only in legacy `SmartCarrier/lib/test.cpp`.
7. Define what "aim at the centre" means mechanically: hover height, contact,
   dwell time, and whether the gripper stays closed throughout numbers 1-3.
8. Confirm whether one PB9 click must automatically clamp and start. The
   current route intentionally requires two clicks for safety.

After these values are supplied, copy the existing SmartCarrier program into
this feature folder, connect the states described in `controller/INTEGRATION.md`,
populate `ArmPoseCalibration.h`, and commission with the chassis raised first.
