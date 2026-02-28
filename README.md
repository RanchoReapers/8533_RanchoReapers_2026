# FRC 2026 — REBUILT
Team 8533 — Rancho Reapers

# LED Colors

The robot uses an addressable LED system to provide visual feedback during operation. Below is a summary of the LED color codes and their meanings:
| **Color** | **Pattern** | **Active** | **Description**
|------------------|-----|--------------|--------------------|
| ![R](https://img.shields.io/badge/-%20-red?style=flat&logo=&logoColor=)![O](https://img.shields.io/badge/-%20-orange?style=flat&logo=&logoColor=)![Y](https://img.shields.io/badge/-%20-yellow?style=flat&logo=&logoColor=)![G](https://img.shields.io/badge/-%20-green?style=flat&logo=&logoColor=)![B](https://img.shields.io/badge/-%20-blue?style=flat&logo=&logoColor=)![I](https://img.shields.io/badge/-%20-indigo?style=flat&logo=&logoColor=)![V](https://img.shields.io/badge/-%20-violet?style=flat&logo=&logoColor=) Rainbow | Rolling | when in AUTONOMOUS | The robot is in AUTONOMOUS mode. |
| ![1](https://img.shields.io/badge/_-FF9A00?style=for-the-badge) Orange | Solid | when CONNECTED to an FMS and DISABLED | The robot does not yet know what alliance it is on. |
| ![2](https://img.shields.io/badge/_-FF0000?style=for-the-badge) Red | Solid | when CONNECTED to an FMS and DISABLED | The robot is on the RED ALLIANCE. |
| ![3](https://img.shields.io/badge/_-0000FF?style=for-the-badge) Blue | Solid | when CONNECTED to an FMS and DISABLED | The robot is on the BLUE ALLIANCE. |
| ![4](https://img.shields.io/badge/_-C300FF?style=for-the-badge) Purple | Pulsing | when CONNECTED to an FMS and in TELEOP | The robot's ALLIANCE HUB is currently active.
| ![5](https://img.shields.io/badge/_-FFFF00?style=for-the-badge) Yellow and ![7](https://img.shields.io/badge/_-808080?style=for-the-badge) Grey | Flashing | when CONNECTED to an FMS and in TELEOP | The robot's ALLIANCE HUB will SWITCH STATES (active/inactive) within FIVE SECONDS. |
| ![6](https://img.shields.io/badge/_-808080?style=for-the-badge) Grey | Solid | when CONNECTED to an FMS and in TELEOP | The robot's ALLIANCE HUB is INACTIVE or the data for the robot's HUB STATE did not send properly. |
| ![7](https://img.shields.io/badge/_-00FF48?style=for-the-badge) Lime | Solid | when CONNECTED to an FMS and in TELEOP | The robot's LIMELIGHT has DETECTED A VALID APRILTAG AT IT'S ALLIANCE HUB and is ACTIVELY AIM ASSISTING THE DRIVER. The driver is NOT TOUCHING the JOYSTICK. |
| ![8](https://img.shields.io/badge/_-00FF48?style=for-the-badge) Lime and ![9](https://img.shields.io/badge/_-870058?style=for-the-badge) Wine | Flashing | when connected to an FMS and in TELEOP | The robot's LIMELIGHT has DETECTED A VALID APRILTAG AT IT'S ALLIANCE HUB and is ACTIVELY AIM ASSISTING THE DRIVER. The driver is FIGHTING AGAINST THE AIM ASSIST. Note that this will not prevent the driver from overriding or continuing to fight against the aim assist.|
| ![R](https://img.shields.io/badge/-%20-red?style=flat&logo=&logoColor=)![O](https://img.shields.io/badge/-%20-orange?style=flat&logo=&logoColor=)![Y](https://img.shields.io/badge/-%20-yellow?style=flat&logo=&logoColor=)![G](https://img.shields.io/badge/-%20-green?style=flat&logo=&logoColor=)![B](https://img.shields.io/badge/-%20-blue?style=flat&logo=&logoColor=)![I](https://img.shields.io/badge/-%20-indigo?style=flat&logo=&logoColor=)![V](https://img.shields.io/badge/-%20-violet?style=flat&logo=&logoColor=) Rainbow | Rolling | when CONNECTED to an FMS and in TELEOP | It is the ENDGAME (<30s left in the match). Note that this can be overridden by limelight-related LED patterns. |
| ![10](https://img.shields.io/badge/_-C300FF?style=for-the-badge) Purple | Solid | when NOT CONNECTED to an FMS and DISABLED | The robot is DISABLED. |
| ![11](https://img.shields.io/badge/_-FFFF00?style=for-the-badge) Yellow and ![11](https://img.shields.io/badge/_-C300FF?style=for-the-badge) Purple | Flashing | when NOT CONNECTED to an FMS and in TELEOP | The robot is in TELEOP and ENABLED (flashes matching the RSL). |

## Notes
- AprilTag Layout (contains corrected positions): https://firstfrc.blob.core.windows.net/frc2026/FieldAssets/2026-field-dimension-dwgs.pdf
      - note that FIRST California uses a WELDED field, not ANDYMARK
- AUTO PATHS are relative to BLUE ALLIANCE -> when selecting in match, they will be relative to whichever alliance the robot is on.

## Controls
Rumble Patterns:
- Endgame Notification: Rumble that increases in intensity as endgame comes closer (between 32-29 seconds remaining in match -- endgame begins at 30s). Will rumble on both controllers.
- Prohibited Action: Rumble for 1 second when requesting an action that is already in progress (i.e. attempting to retract the intake as it is extending). Will rumble only on operator controller.
- Fighting Against Aim Assist: Rumble while driver makes joystick input while aim assist is active (will not prevent the driver from fighting against it).
<img width="1865" height="798" alt="image" src="https://github.com/user-attachments/assets/b5d9171c-c1ac-45c9-90cf-b3bb3f8d3e36" />
<img width="1851" height="795" alt="image" src="https://github.com/user-attachments/assets/af6bd79e-f183-40b6-b5d9-e65d3f7c3eb9" />

## Other
- WPILib docs: https://docs.wpilib.org
- Use GitHub Issues for feature requests and issue tracking
