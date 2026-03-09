package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
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

    Timer shootTimer = new Timer();
    boolean hasSpunUp = false;
    boolean timerHasReset = false;

    public ShooterSubSystem(int shooterLeftCANId, int shooterRightCANId) {
        shooterMotorLeft = new SparkMax(shooterLeftCANId, SparkMax.MotorType.kBrushless);
        shooterMotorRight = new SparkMax(shooterRightCANId, SparkMax.MotorType.kBrushless);

        sparkConfigshooterMotorLeft = new SparkMaxConfig();
        sparkConfigshooterMotorRight = new SparkMaxConfig();

        sparkConfigshooterMotorLeft
                .inverted(false);
        sparkConfigshooterMotorLeft.encoder
                .positionConversionFactor(0.2667)
                .velocityConversionFactor(0.2667);
        sparkConfigshooterMotorLeft.smartCurrentLimit(40, 40);

        sparkConfigshooterMotorRight
                .inverted(false);
        sparkConfigshooterMotorRight.encoder
                .positionConversionFactor(0.3333)
                .velocityConversionFactor(0.3333);
        sparkConfigshooterMotorRight.smartCurrentLimit(40, 40);

        shooterMotorLeft.configure(sparkConfigshooterMotorLeft, com.revrobotics.ResetMode.kResetSafeParameters,
                com.revrobotics.PersistMode.kPersistParameters);
        shooterMotorRight.configure(sparkConfigshooterMotorRight, com.revrobotics.ResetMode.kResetSafeParameters,
                com.revrobotics.PersistMode.kPersistParameters);
        // MAKE SURE TO UPDATE THE POSITION & VELOCITY CONVERSION FACTORS WHEN WE KNOW
        // THE GEAR RATIOS
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
                if (hasSpunUp) {
                    shooterMotorLeft.setVoltage(2.25 * -ShooterConstants.ShooterVoltage);
                    shooterMotorRight.setVoltage(2.25 * ShooterConstants.ShooterVoltage * 0.08);
                } else {
                    if (timerHasReset) {
                        shooterMotorLeft.setVoltage(RobotContainer.operatorController.getRightTriggerAxis() * 2.25  * ShooterConstants.ShooterVoltage);
                        if (shootTimer.hasElapsed(0.3)) {
                            shooterMotorRight.setVoltage(RobotContainer.operatorController.getRightTriggerAxis() * 2.25 * ShooterConstants.ShooterVoltage * 0.08);
                            hasSpunUp = true;
                        }
                    } else {
                        shootTimer.reset();
                        shootTimer.start();
                        timerHasReset = true;

                    }

                }
            } else {
                if (hasSpunUp) {
                    shooterMotorLeft.setVoltage(RobotContainer.operatorController.getRightTriggerAxis() * 2.25 * ShooterConstants.ShooterVoltage);
                    shooterMotorRight.setVoltage(RobotContainer.operatorController.getRightTriggerAxis() * 2.25 * ShooterConstants.ShooterVoltage * 0.08);
                } else {
                    if (timerHasReset) {
                        shooterMotorLeft.setVoltage(RobotContainer.operatorController.getRightTriggerAxis() * 2.25 * ShooterConstants.ShooterVoltage);
                        if (shootTimer.hasElapsed(0.3)) {
                            shooterMotorRight.setVoltage(RobotContainer.operatorController.getRightTriggerAxis() * 2.25 * ShooterConstants.ShooterVoltage * 0.08);
                            hasSpunUp = true;
                        }
                    } else {
                        shootTimer.reset();
                        shootTimer.start();
                        timerHasReset = true;

                    }

                }
            }
        } else {
            hasSpunUp = false;
            timerHasReset = false;
            endShooterMotors();
        }
    }

    public void shooterPeriodic() {
        SmartDashboard.putBoolean("shooterMotorsStopped", shooterMotorsStopped);
    }
}
