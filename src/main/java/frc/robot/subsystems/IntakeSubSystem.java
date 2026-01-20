package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.RobotContainer;

public class IntakeSubSystem extends SubsystemBase {
  boolean doIntake = false;
  boolean intakeMotorsStopped = true;

  SparkMax intakeLeftMotor;
  SparkMaxConfig sparkConfigIntakeLeftMotor;
  SparkMax intakeRightMotor;
  SparkMaxConfig sparkConfigIntakeRightMotor;

  public IntakeSubSystem(int intakeRollersLeftCanId, int intakeRollersRightCanId) {
    intakeLeftMotor = new SparkMax(intakeRollersLeftCanId, SparkMax.MotorType.kBrushless);
    intakeRightMotor = new SparkMax(intakeRollersRightCanId, SparkMax.MotorType.kBrushless);

    sparkConfigIntakeLeftMotor = new SparkMaxConfig();
    sparkConfigIntakeRightMotor = new SparkMaxConfig();

    sparkConfigIntakeLeftMotor
        .idleMode(IdleMode.kBrake)
        .inverted(true);
    sparkConfigIntakeLeftMotor.encoder
        .positionConversionFactor(0.037037037 * Math.PI * 2)
        .velocityConversionFactor(0.037037037 * Math.PI * 2);
    sparkConfigIntakeLeftMotor.smartCurrentLimit(60, 60);

    sparkConfigIntakeRightMotor
        .idleMode(IdleMode.kBrake)
        .inverted(true);
    sparkConfigIntakeRightMotor.encoder
        .positionConversionFactor(0.037037037 * Math.PI * 2)
        .velocityConversionFactor(0.037037037 * Math.PI * 2);
    sparkConfigIntakeRightMotor.smartCurrentLimit(60, 60);
    // MAKE SURE TO UPDATE THE POSITION & VELOCITY CONVERSION FACTORS WHEN WE KNOW THE GEAR RATIOS
  }

  public void endIntakeMotors() {
    intakeLeftMotor.stopMotor();
    intakeRightMotor.stopMotor();
  }

  public void intakeTriggerReleased() {
    if (RobotContainer.operatorController.getLeftTriggerAxis() == 0) {
    intakeMotorsStopped = true;
    }
  }

  public void doIntake() {
    doIntake = true;
    intakeMotorsStopped = false;
  }

  public void intakeControl() {
    if (doIntake == true && intakeMotorsStopped == false) {
      intakeLeftMotor.setVoltage(RobotContainer.operatorController.getLeftTriggerAxis() * 2.25 * IntakeConstants.IntakeVoltage);
      intakeRightMotor.setVoltage(RobotContainer.operatorController.getLeftTriggerAxis() * 2.25 * -IntakeConstants.IntakeVoltage);
      // this may go the wrong direction, switch negatives if true
    } else {
      endIntakeMotors();
    }
  }

  public void intakePeriodic() {
    SmartDashboard.putBoolean("doIntake", doIntake);
    SmartDashboard.putBoolean("intakeMotorsStopped", intakeMotorsStopped);
  }

}
