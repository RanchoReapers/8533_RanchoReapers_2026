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

    public boolean shooterMotorStopped = true;

    SparkMax shooterMotor;
    SparkMaxConfig sparkConfigshooterMotor;

    public ShooterSubSystem(int shooterLeftCanId) {
        shooterMotor = new SparkMax(shooterLeftCanId, SparkMax.MotorType.kBrushless);

        sparkConfigshooterMotor = new SparkMaxConfig();

        sparkConfigshooterMotor
                .inverted(false);
        sparkConfigshooterMotor.encoder
                .positionConversionFactor(0.037037037 * Math.PI * 2)
                .velocityConversionFactor(0.037037037 * Math.PI * 2);
        sparkConfigshooterMotor.smartCurrentLimit(40, 40);

        shooterMotor.configure(sparkConfigshooterMotor, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);
        // MAKE SURE TO UPDATE THE POSITION & VELOCITY CONVERSION FACTORS WHEN WE KNOW THE GEAR RATIOS
    }

    public Command doShootCmd() {
        return new InstantCommand(this::doShoot, this);
    }

    public Command stopShootCmd() {
        return new InstantCommand(this::shooterTriggerReleased, this);
    }

    public void endShooterMotor() {
        shooterMotor.stopMotor();
    }

    public void shooterTriggerReleased() {
        if (RobotContainer.operatorController.getRightTriggerAxis() == 0) {
            shooterMotorStopped = true;
        } else if (DriverStation.isAutonomous()) {
            shooterMotorStopped = true;
        }
    }

    public void doShoot() {
        shooterMotorStopped = false;
    }

    public void shooterControl() {
        if (shooterMotorStopped == false) {
            if (DriverStation.isAutonomous()) {
                shooterMotor.setVoltage(2.25 * ShooterConstants.ShooterVoltage);
            } else {
                shooterMotor.setVoltage(RobotContainer.operatorController.getRightTriggerAxis() * 2.25 * ShooterConstants.ShooterVoltage);
                // this may go the wrong direction, switch negatives if true
            }
        } else {
            endShooterMotor();
        }
    }

    public void shooterPeriodic() {
        SmartDashboard.putBoolean("shooterMotorStopped", shooterMotorStopped);
    }
}
