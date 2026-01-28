# Aim Assist Implementation Guide

## Overview
This document describes the aim assist feature implementation using the Limelight smart camera for the 2026 FRC robot.

## How It Works

### Architecture
The aim assist system consists of three main components:

1. **LimelightDetectionSubSystem** - Processes vision data and calculates corrections
2. **SwerveJoystickCmd** - Blends vision corrections with driver inputs
3. **RobotContainer** - Provides user controls to toggle aim assist

### Detection Logic
The `LimelightDetectionSubSystem` continuously monitors for valid AprilTag targets:
- **Valid Target IDs**: 6-11, 17-22 (speaker/stage targets)
- **Conditions for activation**:
  - Exactly 1 tag in view
  - Target distance < 10 meters
  - Limelight override not active
  - Valid target ID detected

### Correction Algorithm
When a valid target is detected:
- **If `tx > 0.5`**: Robot is too far right → Apply -0.2 correction (move left)
- **If `tx < -0.5`**: Robot is too far left → Apply +0.2 correction (move right)
- **Otherwise**: Robot is centered → No correction needed

The corrections are applied to the robot's lateral (X) speed while the driver retains full control of forward/backward and rotation.

## User Controls

### Driver Controller
- **A Button**: Toggle aim assist on/off
  - When enabled, aim assist will activate automatically when a valid target is detected
  - When disabled, aim assist will not provide any corrections

### SmartDashboard Indicators
- **"Aim Assist Enabled"**: Shows whether the driver has enabled aim assist
- **"aimAssistActive"**: Shows whether the system is currently locked on a target and providing corrections
- **"LimelightX"**: Current horizontal offset from target (tx value)
- **"LimelightY"**: Current vertical offset from target (ty value)
- **"LimelightID"**: Current AprilTag ID being tracked

## Tuning Parameters

The following parameters in `LimelightDetectionSubSystem.java` can be adjusted:

```java
// Line 57-63: Correction deadband and speed
if(tx > 0.5) {              // Deadband threshold (degrees)
    xSpeedLimelight = -0.2;  // Correction speed (m/s)
} else if(tx < -0.5) {
    xSpeedLimelight = 0.2;
} else {
    xSpeedLimelight = 0.0;
}
```

### Recommended Tuning Process
1. Start with small deadband (±0.5°) and low speed (0.2 m/s)
2. Test on field with actual targets
3. Increase speed if corrections are too slow
4. Increase deadband if robot oscillates around target
5. Consider adding PID control for smoother convergence

## Safety Features

1. **Distance Limiting**: Aim assist only activates when target is < 10m away
2. **Target Validation**: Only specific AprilTag IDs are accepted
3. **Override Control**: Limelight override flag can disable aim assist
4. **Additive Corrections**: Driver inputs are not replaced, only augmented
5. **Manual Toggle**: Driver has full control to enable/disable at any time

## Troubleshooting

### Aim Assist Not Activating
- Check SmartDashboard "aimAssistActive" value
- Verify valid target is in view (check "LimelightID")
- Ensure target distance < 10m
- Confirm aim assist is enabled (check "Aim Assist Enabled")

### Robot Oscillates Around Target
- Increase deadband threshold (currently ±0.5°)
- Decrease correction speed (currently 0.2 m/s)
- Consider implementing PID control

### Corrections Too Slow
- Increase correction speed value
- Verify Limelight network connection is stable
- Check for delays in NetworkTables updates

## Future Enhancements

Potential improvements to consider:
1. **PID Control**: Replace bang-bang control with PID for smoother corrections
2. **Distance-based Speed**: Adjust correction speed based on distance to target
3. **Y-axis Corrections**: Add forward/backward corrections for optimal shooting distance
4. **Rotation Assist**: Add auto-rotation to face target directly
5. **Target Prediction**: Use target velocity for moving targets
6. **Multi-target Handling**: Support tracking multiple targets simultaneously
