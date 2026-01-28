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
- **Valid Target IDs**: 10, 26 only
- **Conditions for activation**:
  - Exactly 1 tag in view
  - Target distance < 10 meters
  - Limelight override not active
  - Valid target ID detected

### Correction Algorithm
When a valid target is detected, the system applies two types of corrections:

#### Horizontal Alignment (X-axis)
- **If `tx > 0.5°`**: Robot is too far right → Apply left correction
- **If `tx < -0.5°`**: Robot is too far left → Apply right correction
- **Otherwise**: Robot is centered → No horizontal correction

#### Distance Control (Y-axis)
The robot automatically moves to maintain a target distance from the AprilTag:
- **Target Distance**: Configurable in `LimelightConstants.kTargetDistanceInches` (default: 36 inches)
- **Too Far**: Robot moves forward
- **Too Close**: Robot moves backward
- **Deadband**: ±2 inches to prevent oscillation

The corrections are applied to the robot's lateral (X) and forward/backward (Y) speeds while the driver retains full control of rotation.

## User Controls

### Driver Controller
- **Y Button**: Toggle aim assist on/off
  - When enabled, aim assist will activate automatically when a valid target is detected
  - When disabled, aim assist will not provide any corrections

### SmartDashboard Indicators
- **"Aim Assist Enabled"**: Shows whether the driver has enabled aim assist
- **"aimAssistActive"**: Shows whether the system is currently locked on a target and providing corrections
- **"LimelightX"**: Current horizontal offset from target (tx value)
- **"LimelightY"**: Current vertical offset from target (ty value)
- **"LimelightID"**: Current AprilTag ID being tracked
- **"DistanceToTarget"**: Estimated distance to target in inches
- **"TargetDistance"**: Desired target distance in inches

## Tuning Parameters

The following parameters in `Constants.java` → `LimelightConstants` can be adjusted:

```java
// Target distance from AprilTag in inches when using aim assist
public static final double kTargetDistanceInches = 36.0;

// Valid AprilTag IDs for aim assist (IDs 10 and 26)
public static final long[] kValidTagIDs = {10, 26};

// Correction speeds
public static final double kHorizontalCorrectionSpeed = 0.2; // m/s lateral correction
public static final double kDepthCorrectionSpeed = 0.3; // m/s forward/backward correction

// Deadbands
public static final double kHorizontalDeadbandDegrees = 0.5; // degrees
public static final double kDepthDeadbandInches = 2.0; // inches

// Distance estimation parameters
public static final double kDistanceCalibrationTyReference = -20.0; // ty angle (deg) at target distance
public static final double kMinDistanceInches = 12.0; // Minimum estimated distance
public static final double kMaxDistanceInches = 120.0; // Maximum estimated distance
```

### Recommended Tuning Process
1. **Horizontal Correction Speed**: Start with 0.2 m/s, increase if too slow, decrease if oscillates
2. **Depth Correction Speed**: Start with 0.3 m/s, adjust based on robot response
3. **Target Distance**: Set to optimal shooting/scoring distance (currently 36 inches)
4. **Deadbands**: Increase if robot oscillates, decrease for tighter control
5. **Distance Calibration**: The `kDistanceCalibrationTyReference` should be calibrated:
   - Position robot at exactly `kTargetDistanceInches` from a target
   - Record the `ty` value from SmartDashboard
   - Update `kDistanceCalibrationTyReference` with this value
6. **Distance Limits**: Adjust `kMinDistanceInches` and `kMaxDistanceInches` based on operational needs

## Safety Features

1. **Distance Limiting**: Aim assist only activates when target is < 10m away
2. **Target Validation**: Only specific AprilTag IDs (10, 26) are accepted
3. **Override Control**: Limelight override flag can disable aim assist
4. **Additive Corrections**: Driver inputs are not replaced, only augmented
5. **Manual Toggle**: Driver has full control to enable/disable at any time

## Troubleshooting

### Aim Assist Not Activating
- Check SmartDashboard "aimAssistActive" value
- Verify valid target is in view (check "LimelightID" - should be 10 or 26)
- Ensure target distance < 10m
- Confirm aim assist is enabled (check "Aim Assist Enabled")

### Robot Oscillates Around Target
- Increase horizontal deadband (currently ±0.5°)
- Increase depth deadband (currently ±2 inches)
- Decrease correction speeds
- Consider implementing PID control

### Distance Control Not Working
- Verify "DistanceToTarget" is updating on SmartDashboard
- Calibrate the `estimateDistanceFromTy()` method with actual measurements
- Check that Y-axis corrections are being applied (verify getYSpeedLimelight() returns non-zero)

### Corrections Too Slow or Fast
- Adjust `kHorizontalCorrectionSpeed` and `kDepthCorrectionSpeed` in LimelightConstants
- Test with different values until response feels natural

## Future Enhancements

Potential improvements to consider:
1. **PID Control**: Replace bang-bang control with PID for smoother corrections
2. **Better Distance Estimation**: Use botpose data or actual rangefinder instead of ty estimation
3. **Rotation Assist**: Add auto-rotation to face target directly
4. **Target Prediction**: Use target velocity for moving targets
5. **Adaptive Speeds**: Adjust correction speeds based on error magnitude
6. **Field-Relative Corrections**: Consider alliance color and field orientation
