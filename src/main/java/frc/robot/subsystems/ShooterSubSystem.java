package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.RobotContainer;

public class ShooterSubSystem extends SubsystemBase {
    boolean shooterMotorStopped = true;
    boolean doShoot = false;

    SparkMax shooterMotor;
    SparkMaxConfig sparkConfigShooterMotor;

    public ShooterSubSystem(int shooterCanId) {
        shooterMotor = new SparkMax(shooterCanId, SparkMax.MotorType.kBrushless);

        sparkConfigShooterMotor = new SparkMaxConfig();

        sparkConfigShooterMotor
                .inverted(false);
        sparkConfigShooterMotor.encoder
                .positionConversionFactor(0.037037037 * Math.PI * 2)
                .velocityConversionFactor(0.037037037 * Math.PI * 2);
        sparkConfigShooterMotor.smartCurrentLimit(40, 40);

        // MAKE SURE TO UPDATE THE POSITION & VELOCITY CONVERSION FACTORS WHEN WE KNOW THE GEAR RATIOS
    }

    public void endShooterMotor() {
        shooterMotor.stopMotor();
    }

    public void shooterTriggerReleased() {
        if (RobotContainer.operatorController.getRightTriggerAxis() == 0) {
           shooterMotorStopped = true;
        }
    }

    public void doShoot() {
        doShoot = true;
        shooterMotorStopped = false;
    }

    public void shooterControl() {
        if (doShoot == true && shooterMotorStopped == false) {
            shooterMotor.setVoltage(RobotContainer.operatorController.getRightTriggerAxis() * -2.25 * ShooterConstants.ShooterVoltage);
        } else {
            endShooterMotor();
        }
    }

    public void shooterPeriodic() {
    SmartDashboard.putBoolean("doShoot", doShoot);
    SmartDashboard.putBoolean("shooterMotorStopped", shooterMotorStopped);
  }
}
