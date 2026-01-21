# FRC 2026 — REBUILT
Team 8533 — Rancho Reapers

## Notes
- AprilTag Layout (contains corrected positions): https://firstfrc.blob.core.windows.net/frc2026/FieldAssets/2026-field-dimension-dwgs.pdf
      - note that FIRST California uses a WELDED field, not ANDYMARK

## Controls
Rumble Patterns:
- Endgame Notification: Rumble that increases in intensity as endgame comes closer (between 32-29 seconds remaining in match -- endgame begins at 30s). Will rumble on both controllers.
- Prohibited Action: Rumble for 1 second when requesting an action that is already in progress (i.e. attempting to retract the intake as it is extending). Will rumble only on operator controller.
<img width="1828" height="774" alt="image" src="https://github.com/user-attachments/assets/560fb7a5-8de0-47ba-871e-e394d0f3c7b1" />
<img width="1840" height="783" alt="image" src="https://github.com/user-attachments/assets/9a8a7447-7519-4b81-af2f-f9b55c538627" />




## Subsystem Structure
- Drive (swerve)
- Intake - modify code
- Shooter - create subsystem
- Climber (elevator) - create subsystem
- Vision (Limelight) - modify code

## Other
- WPILib docs: https://docs.wpilib.org
- Use GitHub Issues for feature requests and issue tracking
