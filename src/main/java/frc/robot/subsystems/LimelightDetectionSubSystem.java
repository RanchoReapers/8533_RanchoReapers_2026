package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;

public class LimelightDetectionSubSystem extends SubsystemBase {

    double tagHorizontalOffsetDeg = LimelightHelpers.getTX("limelight");  // Horizontal offset from crosshair to target in degrees
    double tagVerticalOffsetDeg = LimelightHelpers.getTY("limelight");  // Vertical offset from crosshair to target in degrees
    boolean hasTarget = LimelightHelpers.getTV("limelight"); // Do you have a valid target?
    int targetCount = LimelightHelpers.getTargetCount("limelight"); // Get the target count
    double primaryVisibleTagID = LimelightHelpers.getFiducialID("limelight"); // primary visible tag ID
    int currentPipeline = (int) LimelightHelpers.getCurrentPipelineIndex("limelight"); // current pipeline index, 0 = AprilTag, 1 = Fuel

    double distanceOfTagToCameraMeters;
    double currentHeading;
    double headingError;
    double idealDistanceFromTag = 2.154; // in meters

    int primaryTargetID;
    boolean checkedAlly = false;
    public boolean aimAssistActive = false;
    public boolean limelightOverrideActive = false;

    boolean horizontalCorrectionActive = false;
    boolean depthCorrectionActive = false;

    double xSpeedLimelight = 0.0;
    double ySpeedLimelight = 0.0;
    double turnSpeedLimelight = 0.0;

    // modified only from SwerveJoystickCmd but stored here so that they can be accessed by Robot.java
    public double xSpeedBeforeLimelight;
    public double ySpeedBeforeLimelight;
    public double turningSpeedBeforeLimelight;

    double desiredHeading = 0.0; // Dynamically set based on alliance color

    public LimelightDetectionSubSystem() {
    }

    public void updateLimelightData() {
        tagHorizontalOffsetDeg = LimelightHelpers.getTX("limelight");
        tagVerticalOffsetDeg = LimelightHelpers.getTY("limelight");
        hasTarget = LimelightHelpers.getTV("limelight");
        targetCount = LimelightHelpers.getTargetCount("limelight");
        primaryVisibleTagID = LimelightHelpers.getFiducialID("limelight");
        currentPipeline = (int) LimelightHelpers.getCurrentPipelineIndex("limelight");

        if (LimelightHelpers.getRawFiducials("limelight").length > 0) {
            distanceOfTagToCameraMeters = LimelightHelpers.getRawFiducials("limelight")[0].distToCamera;
        }

        currentHeading = RobotContainer.swerveSubsystem.getHeadingInDegrees();
        headingError = desiredHeading - currentHeading;
        headingError = MathUtil.inputModulus(headingError, -180.0, 180.0);
    }

    public void swapLimelightOverrideActive() {
        limelightOverrideActive = !limelightOverrideActive;
    }

    public void aimAssist() {
        updateLimelightData();

        if (DriverStation.isTeleop() && checkedAlly == false) {
            Optional<Alliance> ally = DriverStation.getAlliance();
            if (ally.isPresent()) {
                if (ally.get() == Alliance.Red) {
                    primaryTargetID = 10;
                    LimelightHelpers.setPriorityTagID("limelight", 10);
                    desiredHeading = LimelightConstants.desiredHeadingForRedAlliance;
                } else if (ally.get() == Alliance.Blue) {
                    primaryTargetID = 26;
                    LimelightHelpers.setPriorityTagID("limelight", 26);
                    desiredHeading = MathUtil.inputModulus(LimelightConstants.desiredHeadingForRedAlliance + 180.0, 0.0, 360.0);
                }
                checkedAlly = true;
            } else {
                primaryTargetID = 99999;
            }
        }

        if (currentPipeline == 0 && hasTarget == true && limelightOverrideActive == false && DriverStation.isTeleop() && checkForTagValidity() && distanceOfTagToCameraMeters <= 1.75) {
            if (Math.abs(headingError) > LimelightConstants.headingDeadbandDegrees && primaryTargetID != 99999) {
                turnSpeedLimelight = MathUtil.clamp((LimelightConstants.turnSpeedPerDegree * headingError), -LimelightConstants.maxTurnSpeedRadiansPerSecond, LimelightConstants.maxTurnSpeedRadiansPerSecond);
                // Pause x/y corrections until facing correct heading
                xSpeedLimelight = 0;
                ySpeedLimelight = 0;
                aimAssistActive = true;
                return;
            } else {
                turnSpeedLimelight = 0;
            }

            // x corrections
            if (tagHorizontalOffsetDeg > 5) { // if we are too right (deadband 5 deg)
                horizontalCorrectionActive = true;
                xSpeedLimelight = -LimelightConstants.xyCorrectionSpeedFactor * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
            } else if (tagHorizontalOffsetDeg < -5) { // if we are too left (deadband 5 deg)
                horizontalCorrectionActive = true;
                xSpeedLimelight = LimelightConstants.xyCorrectionSpeedFactor * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
            } else {
                horizontalCorrectionActive = false;
                xSpeedLimelight = 0;
            }

            // y corrections
            if (distanceOfTagToCameraMeters > idealDistanceFromTag + 0.127) { // if we are too far (try to reach roughly 4 feet away from HUB apriltag) (deadband 5 inches (in meters))
                depthCorrectionActive = true;
                ySpeedLimelight = LimelightConstants.xyCorrectionSpeedFactor * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
            } else if (distanceOfTagToCameraMeters < idealDistanceFromTag - 0.127) { // if we are too close (deadband 5 inches (in meters))
                depthCorrectionActive = true;
                ySpeedLimelight = -LimelightConstants.xyCorrectionSpeedFactor * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
            } else {
                depthCorrectionActive = false;
                ySpeedLimelight = 0;
            }

            aimAssistActive = (horizontalCorrectionActive || depthCorrectionActive);

        } else {
            aimAssistActive = false;
            xSpeedLimelight = 0;
            ySpeedLimelight = 0;
            turnSpeedLimelight = 0;
            horizontalCorrectionActive = false;
            depthCorrectionActive = false;
        }

    }

    public boolean checkForTagValidity() {
        return ((primaryTargetID == 99999 && (primaryVisibleTagID == 10 || primaryVisibleTagID == 26)) || (primaryVisibleTagID == primaryTargetID));
        // If we dont know alliance, will return true on both 10 and 26. If we do know alliance, will only return true on the correct tag.
    }

    public double getXSpeedLimelight() {
        return xSpeedLimelight;
    }

    public double getYSpeedLimelight() {
        return ySpeedLimelight;
    }

    public double getTurnSpeedLimelight() {
        return turnSpeedLimelight;
    }

    public int getTargetCountLimelight() {
        return targetCount;
    }

    public boolean getAimAssistActive() {
        return aimAssistActive;
    }

    public void setPipelineIndex(int index) {
        LimelightHelpers.setPipelineIndex("limelight", index);
    }

    public void swapPipelineIndex() {
        if (currentPipeline == 0) {
            setPipelineIndex(1);
        } else {
            setPipelineIndex(0);
        }
    }

    public void periodicOdometry() {
        updateLimelightData();
        SmartDashboard.putNumber("limelightHorizontalOffsetFromTag", tagHorizontalOffsetDeg);
        SmartDashboard.putNumber("limelightVerticalOffsetFromTag", tagVerticalOffsetDeg);
        SmartDashboard.putNumber("limelightID", primaryVisibleTagID);
        SmartDashboard.putBoolean("aimAssistActive", aimAssistActive);
        SmartDashboard.putBoolean("limelightOverrideActive", limelightOverrideActive);
        if (currentPipeline == 0) {
            SmartDashboard.putString("limelightPipelineIndex", "AprilTag (0)");
        } else if (currentPipeline == 1) {
            SmartDashboard.putString("limelightPipelineIndex", "Neural Detector (1)");
        }
        SmartDashboard.putNumber("limelightDistanceOfTagToCameraMeters", distanceOfTagToCameraMeters);
        SmartDashboard.putNumber("limelightTargetCount", targetCount);
        SmartDashboard.putBoolean("limelightHasTarget", hasTarget);
        if (currentPipeline == 1) {
            SmartDashboard.putNumber("Balls Seen", targetCount);
        } else {
            SmartDashboard.putNumber("Balls Seen", 99999);
        }
        SmartDashboard.putNumber("xSpeedLimelight", xSpeedLimelight);
        SmartDashboard.putNumber("ySpeedLimelight", ySpeedLimelight);
        SmartDashboard.putNumber("turnSpeedLimelight", turnSpeedLimelight);
        if (checkForTagValidity()) {
            if (distanceOfTagToCameraMeters > idealDistanceFromTag + 0.127) {
                SmartDashboard.putString("Distance From Hub Alignment", "GO CLOSER");
            } else if (distanceOfTagToCameraMeters < idealDistanceFromTag - 0.127) {
                SmartDashboard.putString("Distance From Hub Alignment", "BACK UP");
            } else {
                SmartDashboard.putString("Distance From Hub Alignment", "ALIGNED");
            }
        } else {
            SmartDashboard.putString("Distance From Hub Alignment", "NOT AT HUB");
        }
    }

    @Override
    public void periodic() {
        updateLimelightData();
        periodicOdometry();
    }
}
