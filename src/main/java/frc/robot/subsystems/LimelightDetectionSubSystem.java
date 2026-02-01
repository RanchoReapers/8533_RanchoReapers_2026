package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj.DriverStation;

public class LimelightDetectionSubSystem extends SubsystemBase {

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    double tx = table.getEntry("tx").getDouble(0.0);
    double ty = table.getEntry("ty").getDouble(0.0);
    double ta = table.getEntry("ta").getDouble(0.0);

    long tid = table.getEntry("tid").getInteger(0);

    boolean aimAssistActive = false;
    boolean limelightOverrideActive = false;

    double[] botpose = table.getEntry("botpose").getDoubleArray(new double[0]);
    double[] testTable = {1, 2, 3, 4, 5, 6};

    double tagsInView = testTable[3];
    double tagAveDistance = testTable[5];

    double xSpeedLimelight = 0.0;
    double ySpeedLimelight = 0.0;

    double turnAngleLimelight = 0.0;

    public LimelightDetectionSubSystem() {
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

    public void limelightOverrideActive() {
        limelightOverrideActive = !limelightOverrideActive;
    }

    public void aimAssist() {
        updateLimelightData();
        if (limelightOverrideActive == false && DriverStation.isTeleop() && (tid == 10 || tid == 26) && tagsInView <= 3 && tagAveDistance < 10) {
            aimAssistActive = true;
            if (tx > 0.5) { // if we are too right
                xSpeedLimelight = -0.2;
            } else if (tx < 0.5) { // if we are too left
                xSpeedLimelight = 0.2;
            }
        } else {
            aimAssistActive = false;
            xSpeedLimelight = 0;
        }
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

    public void periodicOdometry() {
        SmartDashboard.putNumber("LimelightX", tx);
        SmartDashboard.putNumber("LimelightY", ty);
        SmartDashboard.putNumber("LimelightArea", ta);
        SmartDashboard.putNumber("LimelightID", tid);
        SmartDashboard.putBoolean("aimAssistActive", aimAssistActive);
        SmartDashboard.putBoolean("limelightOverrideActive", limelightOverrideActive);
    }

}
