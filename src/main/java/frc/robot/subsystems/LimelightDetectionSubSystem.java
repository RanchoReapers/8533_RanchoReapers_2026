package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;
import java.util.function.BooleanSupplier;

public class LimelightDetectionSubSystem extends SubsystemBase{

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    double tx = table.getEntry("tx").getDouble(0.0);
    double ty = table.getEntry("ty").getDouble(0.0);
    double ta = table.getEntry("ta").getDouble(0.0);

    long tid = table.getEntry("tid").getInteger(0);

    boolean aimAssistActive = false;
    boolean limelightOverride = false;

    double[] botpose = table.getEntry("botpose").getDoubleArray(new double[0]);
    double[] testTable = {1,2,3,4,5,6};

    double tagsInView = testTable[3];
    double tagAveDistance = testTable[5];

    double xSpeedLimelight = 0.0;
    double ySpeedLimelight = 0.0;

    double turnAngleLimelight = 0.0;
    
    // Distance to target in inches (calculated from ty and known tag height)
    double distanceToTargetInches = 0.0;

    public LimelightDetectionSubSystem() {
    }

    public Command activateLimelightOverride() {
        limelightOverride = !limelightOverride;
        return new InstantCommand();
    }
  
    public void updateLimelightData() {
        tx = -table.getEntry("tx").getDouble(0.0);
        ty = -table.getEntry("ty").getDouble(0.0);
        ta = table.getEntry("ta").getDouble(0.0);
        tid = table.getEntry("tid").getInteger(0);
        botpose = table.getEntry("botpose").getDoubleArray(new double[0]);
        tagsInView = testTable[3];
        tagAveDistance = testTable[5];
    }

    public void aimAssist() {
        updateLimelightData();

        // Check if we have a valid target ID (10 or 26)
        boolean validTarget = (tid == 10 || tid == 26);

        if(validTarget && tagsInView == 1 && limelightOverride == false && tagAveDistance < 10) {
            aimAssistActive = true;
            
            // Horizontal alignment (X-axis correction)
            if(tx > LimelightConstants.kHorizontalDeadbandDegrees) { // if we are too right
                xSpeedLimelight = -LimelightConstants.kHorizontalCorrectionSpeed;
            } else if(tx < -LimelightConstants.kHorizontalDeadbandDegrees) { // if we are too left
                xSpeedLimelight = LimelightConstants.kHorizontalCorrectionSpeed;
            } else {
                xSpeedLimelight = 0.0;
            }
            
            // Distance control (Y-axis correction) - maintain target distance
            // Robot moves forward when too far, backward when too close
            // Calculate approximate distance based on botpose or use a simple ty-based estimation
            // Note: This is a simplified approach. In production, use actual distance from botpose
            distanceToTargetInches = estimateDistanceFromTy(ty);
            
            double distanceError = distanceToTargetInches - LimelightConstants.kTargetDistanceInches;
            
            if (distanceError < -LimelightConstants.kDepthDeadbandInches) {
                // Too far - move forward
                ySpeedLimelight = LimelightConstants.kDepthCorrectionSpeed;
            } else if (distanceError > LimelightConstants.kDepthDeadbandInches) {
                // Too close - move backward
                ySpeedLimelight = -LimelightConstants.kDepthCorrectionSpeed;
            } else {
                ySpeedLimelight = 0.0;
            }
        } else {
            aimAssistActive = false;
            xSpeedLimelight = 0;
            ySpeedLimelight = 0;
        }
    }
    
    /**
     * Estimates distance to target in inches based on vertical angle (ty).
     * This is a simplified estimation. For better accuracy, use botpose data.
     * Assumes camera mounted at a fixed height and angle.
     * 
     * @param ty The vertical angle to the target in degrees (negative when looking up at target)
     * @return Estimated distance to target in inches
     */
    private double estimateDistanceFromTy(double ty) {
        // Handle edge case where ty is zero or very small
        if (Math.abs(ty) < 0.1) {
            return LimelightConstants.kMaxDistanceInches; // Default far distance if no/minimal angle
        }
        
        // Simple estimation using inverse relationship with ty
        // Formula: distance (inches) = kTargetDistanceInches * (kDistanceCalibrationTyReference / ty)
        // This assumes ty is negative when looking up at target (typical Limelight configuration)
        // Calibration: When ty = kDistanceCalibrationTyReference, distance = kTargetDistanceInches
        double estimatedDistance = LimelightConstants.kTargetDistanceInches * 
                                   (LimelightConstants.kDistanceCalibrationTyReference / ty);
        
        // Clamp to reasonable range
        return Math.max(LimelightConstants.kMinDistanceInches, 
                       Math.min(LimelightConstants.kMaxDistanceInches, estimatedDistance));
    }

    public double getXSpeedLimelight() {
        return xSpeedLimelight;
    }

    public double getYSpeedLimelight() {
        return ySpeedLimelight;
    }

    public double getTurnAngleLimelight() {
        return turnAngleLimelight;
    }

    public BooleanSupplier getAimAssistActive() {
        return () -> aimAssistActive;
    }

    @Override
    public void periodic() {
        aimAssist();
        periodicOdometry();
    }

    public void periodicOdometry() {
        SmartDashboard.putNumber("LimelightX", tx);
        SmartDashboard.putNumber("LimelightY", ty);
        SmartDashboard.putNumber("LimelightArea", ta);
        SmartDashboard.putNumber("LimelightID", tid);
        SmartDashboard.putBoolean("aimAssistActive", aimAssistActive);
        SmartDashboard.putNumber("DistanceToTarget", distanceToTargetInches);
        SmartDashboard.putNumber("TargetDistance", LimelightConstants.kTargetDistanceInches);
    }

}

