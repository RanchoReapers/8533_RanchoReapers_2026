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
  boolean intakeMotorStopped = true;

  SparkMax intakeMotor;
  SparkMaxConfig sparkConfigIntakeMotor;

  public IntakeSubSystem(int intakeRollersCanId) {
    intakeMotor = new SparkMax(intakeRollersCanId, SparkMax.MotorType.kBrushless);

    sparkConfigIntakeMotor = new SparkMaxConfig();

    sparkConfigIntakeMotor
        .idleMode(IdleMode.kBrake)
        .inverted(true);
    sparkConfigIntakeMotor.encoder
        .positionConversionFactor(0.037037037 * Math.PI * 2)
        .velocityConversionFactor(0.037037037 * Math.PI * 2);
    sparkConfigIntakeMotor.smartCurrentLimit(60, 60);
    // MAKE SURE TO UPDATE THE POSITION & VELOCITY CONVERSION FACTORS WHEN WE KNOW THE GEAR RATIOS
  }

  public void endIntakeMotor() {
    intakeMotor.stopMotor();
  }

  public void intakeTriggerReleased() {
    if (RobotContainer.operatorController.getLeftTriggerAxis() == 0) {
    intakeMotorStopped = true;
    }
  }

  public void doIntake() {
    doIntake = true;
    intakeMotorStopped = false;
  }

  public void intakeControl() {
    if (doIntake == true && intakeMotorStopped == false) {
      intakeMotor.setVoltage(RobotContainer.operatorController.getLeftTriggerAxis() * 2.25 * IntakeConstants.IntakeVoltage);
    } else {
      endIntakeMotor();
    }
  }

  public void intakePeriodic() {
    SmartDashboard.putBoolean("doIntake", doIntake);
    SmartDashboard.putBoolean("intakeMotorStopped", intakeMotorStopped);
  }

}
