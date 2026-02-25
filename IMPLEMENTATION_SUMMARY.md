# Implementation Summary: Neural Detector Pipeline for FRC Robot

## Date: 2026-02-25

## Problem Statement
Add a neural detector pipeline to detect fuel balls in the robot's hopper and use that information to intelligently control shooting duration during autonomous operation.

## Requirements Met ✅

1. ✅ **Neural Detector Pipeline**: Implemented pipeline 1 for fuel/ball detection
   - Detects objects with class ID 0 (fuel balls)
   - Monitors confidence threshold (≥0.5)
   - Reads from `tclass` NetworkTable entry
   - Counts balls in hopper view

2. ✅ **Auto Shooting Logic**: SmartShootCommand implementation
   - Monitors ball count during shooting
   - If ≤2 balls: shoots for 1 more second then stops
   - If 5 seconds elapsed: stops regardless of ball count
   - Automatically manages pipeline switching

3. ✅ **Teleop AprilTag Aim Assist**: Maintained and enhanced
   - Only active in teleop mode with AprilTag pipeline
   - Targets IDs 10 and 26
   - Provides horizontal alignment and distance control

4. ✅ **Pipeline Switching**: Driver can toggle between modes
   - X button on driver controller
   - Switches between AprilTag (0) and Neural (1)
   - Shows current mode on SmartDashboard

5. ✅ **Teleop Neural Mode**: Read-only ball count display
   - No aim assist in neural mode
   - Displays `DetectedBallCount` on SmartDashboard
   - For debugging and monitoring

## Files Created

### src/main/java/frc/robot/commands/SmartShootCommand.java
Custom command that manages intelligent shooting:
- Monitors ball count in real-time
- Implements conditional stopping logic
- Handles pipeline switching automatically
- Integrates with shooter subsystem

### NEURAL_DETECTOR_GUIDE.md
Comprehensive documentation covering:
- Feature overview and behavior
- Component descriptions
- Usage instructions
- Configuration requirements
- Tuning parameters
- SmartDashboard outputs
- Benefits and future enhancements

## Files Modified

### src/main/java/frc/robot/Constants.java
Added LimelightConstants:
- Pipeline indices (0 = AprilTag, 1 = Neural)
- Neural detector settings (class ID, confidence threshold)
- Auto shooting parameters (max time, extra time, ball threshold)

### src/main/java/frc/robot/subsystems/LimelightDetectionSubSystem.java
Enhanced with:
- Pipeline switching methods
- Neural detector data reading
- Ball counting logic
- Pipeline state tracking
- SmartDashboard outputs for mode and ball count
- Teleop-only aim assist restriction

### src/main/java/frc/robot/RobotContainer.java
Updates:
- Added X button binding for pipeline toggle
- Updated Autos constructor to include limelight subsystem

### src/main/java/frc/robot/commands/Autos.java
Enhancements:
- Added limelight subsystem parameter
- Added SmartShootCommand integration
- Updated example auto routines (LEFT_TRENCH, LEFT_BUMP)
- Added pipeline switch bindings for Choreo

## Key Features

### SmartShootCommand Logic
```
Initialize:
  - Start timer
  - Switch to neural pipeline

Execute (periodic):
  - Read ball count
  - If count ≤ 2 and not yet flagged:
    - Flag low ball count
    - Record timestamp

IsFinished:
  - Return true if:
    - 5 seconds elapsed OR
    - Low balls flagged AND 1 second since flag

End:
  - Stop shooter
  - Switch to AprilTag pipeline
```

### Control Mapping
**Driver Controller:**
- Y: Toggle aim assist (AprilTag mode only)
- X: Toggle pipeline (AprilTag ↔ Neural)
- RB: Field-oriented drive

**Operator Controller:**
- LT: Intake
- RT: Shoot
- X: Intake retractor

### SmartDashboard Outputs
- `CurrentPipeline`: 0 or 1
- `PipelineMode`: "AprilTag" or "Neural"
- `DetectedBallCount`: Number of balls detected
- `aimAssistActive`: Boolean
- All standard limelight data (tx, ty, ta, tid, etc.)

## Configuration Requirements

### Limelight Setup
1. **Pipeline 0 (AprilTag)**:
   - Enable AprilTag detector
   - Configure for IDs 10, 26
   - Set appropriate thresholds

2. **Pipeline 1 (Neural)**:
   - Enable Neural Detector
   - Train/load model for fuel detection
   - Ensure balls classified as class ID 0
   - Position camera to view hopper

### Tuning Parameters
In `Constants.LimelightConstants`:
- `kMinDetectionConfidence = 0.5`: Adjust based on detection accuracy
- `kMinBallsForContinuedShooting = 2`: Threshold for "low" count
- `kMaxAutoShootingTime = 5.0`: Maximum shooting duration
- `kExtraShootingTime = 1.0`: Additional time when low

## Testing Checklist

- [ ] Verify pipeline switching with X button
- [ ] Confirm aim assist only works in AprilTag mode
- [ ] Test ball counting accuracy in various conditions
- [ ] Validate SmartShootCommand with 0, 1, 2, 3+ balls
- [ ] Confirm 5-second maximum time limit
- [ ] Check pipeline auto-switching in auto modes
- [ ] Test SmartDashboard value updates
- [ ] Verify autonomous routine execution

## Branch Information
- Branch: `copilot/add-aim-assist-limelight`
- Latest commit: Implements SmartShootCommand and neural detector pipeline
- Status: Ready for robot testing

## Notes
- Main branch has unrelated history - could not merge directly
- Implementation is self-contained and doesn't conflict with main branch changes
- All existing aim assist functionality preserved and enhanced
- Documentation provided for future maintenance and tuning

## Future Improvements
- Add PID control for smoother aim assist
- Implement ball trajectory prediction
- Use botpose for more accurate distance measurement
- Add multi-target tracking capabilities
- Enhance confidence filtering for robust detection
- Add visual indicators for pipeline mode on driver station
