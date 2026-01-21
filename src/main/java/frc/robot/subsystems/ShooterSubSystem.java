package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.RobotContainer;

public class ShooterSubSystem extends SubsystemBase {
    boolean shooterMotorsStopped = true;

    SparkMax shooterLeftMotor;
    SparkMaxConfig sparkConfigShooterLeftMotor;
    SparkMax shooterRightMotor;
    SparkMaxConfig sparkConfigShooterRightMotor;

    public ShooterSubSystem(int shooterLeftCanId, int shooterRightCanId) {
        shooterLeftMotor = new SparkMax(shooterLeftCanId, SparkMax.MotorType.kBrushless);
        shooterRightMotor = new SparkMax(shooterRightCanId, SparkMax.MotorType.kBrushless);

        sparkConfigShooterLeftMotor = new SparkMaxConfig();
        sparkConfigShooterRightMotor = new SparkMaxConfig();

        sparkConfigShooterLeftMotor
                .inverted(false);
        sparkConfigShooterLeftMotor.encoder
                .positionConversionFactor(0.037037037 * Math.PI * 2)
                .velocityConversionFactor(0.037037037 * Math.PI * 2);
        sparkConfigShooterLeftMotor.smartCurrentLimit(40, 40);

        sparkConfigShooterRightMotor
                .inverted(false);
        sparkConfigShooterRightMotor.encoder
                .positionConversionFactor(0.037037037 * Math.PI * 2)
                .velocityConversionFactor(0.037037037 * Math.PI * 2);
        sparkConfigShooterRightMotor.smartCurrentLimit(40, 40);
        // MAKE SURE TO UPDATE THE POSITION & VELOCITY CONVERSION FACTORS WHEN WE KNOW THE GEAR RATIOS
    }

    public void endShooterMotors() {
        shooterLeftMotor.stopMotor();
        shooterRightMotor.stopMotor();
    }

    public void shooterTriggerReleased() {
        if (RobotContainer.operatorController.getRightTriggerAxis() == 0) {
           shooterMotorsStopped = true;
        }
    }

    public void doShoot() {
        shooterMotorsStopped = false;
    }

    public void shooterControl() {
        if (shooterMotorsStopped == false) {
            shooterLeftMotor.setVoltage(RobotContainer.operatorController.getRightTriggerAxis() * 2.25 * ShooterConstants.ShooterVoltage);
            shooterRightMotor.setVoltage(RobotContainer.operatorController.getRightTriggerAxis() * 2.25 * -ShooterConstants.ShooterVoltage);
            // this may go the wrong direction, switch negatives if true
        } else {
            endShooterMotors();
        }
    }

    public void shooterPeriodic() {
        SmartDashboard.putBoolean("shooterMotorsStopped", shooterMotorsStopped);
    }
}
