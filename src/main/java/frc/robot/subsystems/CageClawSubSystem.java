package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CageClawConstants;
import com.revrobotics.RelativeEncoder;

public class CageClawSubSystem extends SubsystemBase {

  SparkMax cageClawMotor;
  SparkMaxConfig sparkConfigCageClawMotor;

  RelativeEncoder clawEncoder;

  boolean clawOpen = false;
  // lockouts to prevent user switching arm state too often
  boolean clawInUseOpening = false; // Arm is currently being used to move open
  boolean clawInUseClosing = false; // Arm is currently being used to move closed


  public CageClawSubSystem(int clampOpenCanId) {

    cageClawMotor = new SparkMax(clampOpenCanId, SparkMax.MotorType.kBrushless);

    sparkConfigCageClawMotor = new SparkMaxConfig();

    clawEncoder = cageClawMotor.getEncoder();

    sparkConfigCageClawMotor
        .idleMode(IdleMode.kBrake)
        .inverted(false);
    sparkConfigCageClawMotor.encoder
        .positionConversionFactor(0.0133333 * Math.PI * 2)
        .velocityConversionFactor(0.0133333 * Math.PI * 2);
    sparkConfigCageClawMotor.smartCurrentLimit(60, 60);

    cageClawMotor.configure(sparkConfigCageClawMotor, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  public void endClawMotors() {
    cageClawMotor.stopMotor();
  }

  public void switchClawOpen() {
    if (clawInUseClosing == false && clawInUseOpening == false) {
      clawOpen = !clawOpen;
    }
  }

  public void clampControl() {
    if (clawOpen == true && clawInUseClosing == false) {
      if (clawEncoder.getPosition() <= 50 * Math.PI / 180) {
        clawInUseOpening = true;
        cageClawMotor.setVoltage(CageClawConstants.CageClawVoltage);
      } else {
        endClawMotors();
        clawInUseOpening = false;
      }
    } else if (clawOpen == false && clawInUseOpening == false) {
      if (clawEncoder.getPosition() >= 0 * Math.PI / 180) {
        clawInUseClosing = true;
        cageClawMotor.setVoltage(-CageClawConstants.CageClawVoltage);
      } else {
        endClawMotors();
        clawInUseClosing = false;
      }
    }
  }

  public void periodicOdometry() {
    SmartDashboard.putBoolean("clawOpen", clawOpen);
    SmartDashboard.putBoolean("clawInUseClosing", clawInUseClosing);
    SmartDashboard.putBoolean("clawInUseOpening", clawInUseOpening);

    SmartDashboard.putNumber("CLAW motor position", clawEncoder.getPosition() * 180/Math.PI);
  }

}