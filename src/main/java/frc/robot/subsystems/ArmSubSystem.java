package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmSubSystem extends SubsystemBase {

  SparkMax armDriveLeft;
  SparkMax armDriveRight;

  SparkMaxConfig sparkConfigDriveRight;
  SparkMaxConfig sparkConfigDriveLeft;
//change
  RelativeEncoder armEncoderLeft;
  RelativeEncoder armEncoderRight;

  boolean armLow = true; // where you are TRYING to go
  
  // lockouts to prevent user switching arm state too often
  boolean armInUseDown = false; 
  boolean armInUseUp = false;

  boolean armForClawInUse = false;
  boolean armForClawRequested = false;

  public ArmSubSystem(int armLeftCANId, int armRightCANId) {
    armDriveLeft = new SparkMax(armLeftCANId, SparkMax.MotorType.kBrushless);
    armDriveRight = new SparkMax(armRightCANId, SparkMax.MotorType.kBrushless);

    sparkConfigDriveRight = new SparkMaxConfig();
    sparkConfigDriveLeft = new SparkMaxConfig();

    armEncoderLeft = armDriveLeft.getEncoder();
    armEncoderRight = armDriveRight.getEncoder();

    sparkConfigDriveLeft
        .idleMode(IdleMode.kBrake)
        .inverted(true);
    sparkConfigDriveLeft.encoder
        .positionConversionFactor(0.003047619 * Math.PI * 2)
        .velocityConversionFactor(0.003047619 * Math.PI * 2);
    sparkConfigDriveLeft.smartCurrentLimit(60, 60);

    sparkConfigDriveRight
        .idleMode(IdleMode.kBrake)
        .inverted(false);
    sparkConfigDriveRight.encoder
        .positionConversionFactor(0.003047619 * Math.PI * 2)
        .velocityConversionFactor(0.003047619 * Math.PI * 2);
    sparkConfigDriveRight.smartCurrentLimit(60, 60);

    armDriveLeft.configure(sparkConfigDriveLeft, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    armDriveRight.configure(sparkConfigDriveRight, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void endArmMotors() {
    armDriveLeft.stopMotor();
    armDriveRight.stopMotor();
  }

  public void switchArmLow() {
    if (armInUseUp == false && armInUseDown == false && armForClawInUse == false) {
      armLow = !armLow;
    }
  }

  public void switchClawArmLow() {
    if (armForClawRequested == false && armForClawInUse == false && armInUseUp == false && armInUseDown == false) {
      armForClawRequested = true;
    } else if (armForClawRequested == true && armForClawInUse == false && armInUseUp == false && armInUseDown == false) {
      armLow = true;
      armInUseUp = false;
      armInUseDown = false;
      armForClawRequested = false;
    }
  }

  public void armControl3State() {
    // set the number of degrees to be one lower/higher depending on direction for movement to allow for stopping time
    if (armForClawRequested == true) {
      if (armEncoderLeft.getPosition() <= 30 * Math.PI / 180) {
        armForClawInUse = true;
        armDriveLeft.setVoltage(ArmConstants.ArmVoltage);
        armDriveRight.setVoltage(ArmConstants.ArmVoltage);
      } else {
        endArmMotors();
        armForClawInUse = false;
      }
    } else if (armLow == true && armInUseUp == false) {
      if (armEncoderLeft.getPosition() >= -56.6 * Math.PI / 180) { // higher
        armInUseDown = true;
        armDriveLeft.setVoltage(-8);
        armDriveRight.setVoltage(-8);
      } else {
        endArmMotors();
        armInUseDown = false;
      }
    } else if (armLow == false && armInUseDown == false) {
      if (armEncoderLeft.getPosition() <= -15 * Math.PI / 180) { // lower
        armInUseUp = true;
        armDriveLeft.setVoltage(ArmConstants.ArmVoltage);
        armDriveRight.setVoltage(ArmConstants.ArmVoltage);
      } else {
        endArmMotors();
        armInUseUp = false;
      }
    }
  }

  public void periodicOdometry() {
    SmartDashboard.putBoolean("armLow", armLow);
    SmartDashboard.putBoolean("armInUseUp", armInUseUp);
    SmartDashboard.putBoolean("armInUseDown", armInUseDown);

    SmartDashboard.putNumber("Left ARM motor position", armEncoderLeft.getPosition() * 180 / Math.PI);
    SmartDashboard.putNumber("Right ARM motor position", armEncoderRight.getPosition() * 180 / Math.PI);
  }

}