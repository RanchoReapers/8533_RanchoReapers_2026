package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.RobotContainer;

public class IntakeSubSystem extends SubsystemBase {

    public boolean intakeMotorStopped = true;

    SparkMax intakeMotor;
    SparkMaxConfig sparkConfigIntakeMotor;

    public IntakeSubSystem(int intakeRollersLeftCanId) {
        intakeMotor = new SparkMax(intakeRollersLeftCanId, SparkMax.MotorType.kBrushless);

        sparkConfigIntakeMotor = new SparkMaxConfig();

        sparkConfigIntakeMotor
                .idleMode(IdleMode.kBrake)
                .inverted(true);
        sparkConfigIntakeMotor.encoder
                .positionConversionFactor(9 * Math.PI * 2)
                .velocityConversionFactor((9 * Math.PI * 2) / 60 );
        sparkConfigIntakeMotor.smartCurrentLimit(60, 60);
        // MAKE SURE TO UPDATE THE POSITION & VELOCITY CONVERSION FACTORS WHEN WE KNOW THE GEAR RATIOS

        intakeMotor.configure(sparkConfigIntakeMotor, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);
    }

    public Command doIntakeCmd() {
        return new InstantCommand(this::doIntake, this);
    }

    public Command stopIntakeCmd() {
        return new InstantCommand(this::intakeTriggerReleased, this);
    }

    public void endIntakeMotor() {
        intakeMotor.stopMotor();
    }

    public void intakeTriggerReleased() {
        if (RobotContainer.operatorController.getLeftTriggerAxis() == 0) {
            intakeMotorStopped = true;
        } else if (DriverStation.isAutonomous()) {
            intakeMotorStopped = true;
        }
    }

    public void doIntake() {
        intakeMotorStopped = false;
    }

    public void intakeControl() {
        if (intakeMotorStopped == false) {
            if (DriverStation.isAutonomous()) {
                intakeMotor.setVoltage(2.25 * IntakeConstants.IntakeVoltage);
            } else {
                intakeMotor.setVoltage(RobotContainer.operatorController.getLeftTriggerAxis() * 2.25 * IntakeConstants.IntakeVoltage);
                // this may go the wrong direction, switch negatives if true
            }
        } else {
            endIntakeMotor();
        }
    }

    public void intakePeriodic() {
        SmartDashboard.putBoolean("intakeMotorStopped", intakeMotorStopped);
    }

}
