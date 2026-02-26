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

    SparkMax shooterMotorLeft;
    SparkMax shooterMotorRight;

    SparkMaxConfig sparkConfigshooterMotorLeft;
    SparkMaxConfig sparkConfigshooterMotorRight;

    public ShooterSubSystem(int shooterLeftCANId, int shooterRightCANId) {
        shooterMotorLeft = new SparkMax(shooterLeftCANId, SparkMax.MotorType.kBrushless);
        shooterMotorRight = new SparkMax(shooterLeftCANId, SparkMax.MotorType.kBrushless);

        sparkConfigshooterMotorLeft = new SparkMaxConfig();
        sparkConfigshooterMotorRight = new SparkMaxConfig();

        sparkConfigshooterMotorLeft
                .inverted(false);
        sparkConfigshooterMotorLeft.encoder
                .positionConversionFactor(0.037037037 * Math.PI * 2)
                .velocityConversionFactor(0.037037037 * Math.PI * 2);
        sparkConfigshooterMotorLeft.smartCurrentLimit(40, 40);

        sparkConfigshooterMotorRight
                .inverted(false);
        sparkConfigshooterMotorRight.encoder
                .positionConversionFactor(0.037037037 * Math.PI * 2)
                .velocityConversionFactor(0.037037037 * Math.PI * 2);
        sparkConfigshooterMotorRight.smartCurrentLimit(40, 40);

        shooterMotorLeft.configure(sparkConfigshooterMotorLeft, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);
        shooterMotorRight.configure(sparkConfigshooterMotorRight, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);
        // MAKE SURE TO UPDATE THE POSITION & VELOCITY CONVERSION FACTORS WHEN WE KNOW THE GEAR RATIOS
    }

    public Command doShootCmd() {
        return new InstantCommand(this::doShoot, this);
    }

    public Command stopShootCmd() {
        return new InstantCommand(this::shooterTriggerReleased, this);
    }

    public void endShooterMotors() {
        shooterMotorLeft.stopMotor();
        shooterMotorRight.stopMotor();
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
                shooterMotorLeft.setVoltage(2.25 * ShooterConstants.ShooterVoltage);
                shooterMotorRight.setVoltage(2.25 * -ShooterConstants.ShooterVoltage);
            } else {
                shooterMotorLeft.setVoltage(RobotContainer.operatorController.getRightTriggerAxis() * 2.25 * ShooterConstants.ShooterVoltage);
                shooterMotorRight.setVoltage(RobotContainer.operatorController.getRightTriggerAxis() * 2.25 * -ShooterConstants.ShooterVoltage);
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
