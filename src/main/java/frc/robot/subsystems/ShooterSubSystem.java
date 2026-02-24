package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DriverStation;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.RobotContainer;

public class ShooterSubSystem extends SubsystemBase {

    public boolean shooterMotorsStopped = true;

    SparkMax shooterMotorBottom;
    SparkMax shooterMotorTop;

    SparkMaxConfig sparkConfigShooterMotorBottom;
    SparkMaxConfig sparkConfigShooterMotorTop;

    public ShooterSubSystem(int shooterBottomCanId, int shooterTopCanId) {
        shooterMotorBottom = new SparkMax(shooterBottomCanId, SparkMax.MotorType.kBrushless);
        shooterMotorTop = new SparkMax(shooterTopCanId, SparkMax.MotorType.kBrushless);

        sparkConfigShooterMotorBottom = new SparkMaxConfig();
        sparkConfigShooterMotorTop = new SparkMaxConfig();

        sparkConfigShooterMotorBottom
                .inverted(false);
        sparkConfigShooterMotorBottom.encoder
                .positionConversionFactor(0.037037037 * Math.PI * 2)
                .velocityConversionFactor(0.037037037 * Math.PI * 2);
        sparkConfigShooterMotorBottom.smartCurrentLimit(40, 40);

        sparkConfigShooterMotorTop
                .inverted(false);
        sparkConfigShooterMotorTop.encoder
                .positionConversionFactor(0.037037037 * Math.PI * 2)
                .velocityConversionFactor(0.037037037 * Math.PI * 2);
        sparkConfigShooterMotorTop.smartCurrentLimit(40, 40);

        shooterMotorBottom.configure(sparkConfigShooterMotorBottom, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);
        shooterMotorTop.configure(sparkConfigShooterMotorTop, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);
        // MAKE SURE TO UPDATE THE POSITION & VELOCITY CONVERSION FACTORS WHEN WE KNOW THE GEAR RATIOS
    }

    public Command doShootCmd() {
        return new InstantCommand(this::doShoot, this);
    }

    public Command stopShootCmd() {
        return new InstantCommand(this::shooterTriggerReleased, this);
    }

    public void endShooterMotors() {
        shooterMotorBottom.stopMotor();
        shooterMotorTop.stopMotor();
    }

    public void shooterTriggerReleased() {
        if (RobotContainer.operatorController.getRightTriggerAxis() == 0) {
            shooterMotorsStopped = true;
        } else if (DriverStation.isAutonomous()) {
            shooterMotorsStopped = true;
        }
    }

    public void doShoot() {
        shooterMotorsStopped = false;
    }

    public void shooterControl() {
        if (shooterMotorsStopped == false) {
            if (DriverStation.isAutonomous()) {
                shooterMotorBottom.setVoltage(2.25 * ShooterConstants.ShooterVoltage);
                shooterMotorTop.setVoltage(2.25 * -ShooterConstants.ShooterVoltage);
            } else {
                shooterMotorBottom.setVoltage(RobotContainer.operatorController.getRightTriggerAxis() * 2.25 * ShooterConstants.ShooterVoltage);
                shooterMotorTop.setVoltage(RobotContainer.operatorController.getRightTriggerAxis() * 2.25 * -ShooterConstants.ShooterVoltage);
                // this may go the wrong direction, switch negatives if true
            }
        } else {
            endShooterMotors();
        }
    }

    public void shooterPeriodic() {
        SmartDashboard.putBoolean("shooterMotorsStopped", shooterMotorsStopped);
    }
}
