package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.IntakeRetractorConstants;

public class IntakeRetractorSubSystem extends SubsystemBase {

    SparkMax intakeRetractorMotor;
    SparkMaxConfig sparkConfigIntakeRetractorMotor;

    private final CANcoder intakeRetractorAbsoluteEncoder;

    private final Timer intakeRetractionProhibitedRumbleTimer = new Timer();
    public boolean intakeRetractionProhibitedRumbleActive = false;

    // Info alert to indicate when the intake is retracted or extended
    private final Alert intakeRetractorStatusAlert = new Alert("Intake Retractor status", AlertType.kInfo);

    boolean intakeRetractorMotorStopped = true;

    public enum currentStateIntakeRetractor {
        IDLE_RETRACTED,
        IDLE_EXTENDED,
        EXTENDING
    }

    private static final double kRetractedAngle = -90.0;
    private static final double kMotorCutoffAngleForExtension = -30; // angle at which the motor should stop applying voltage when extending. this is because mechanically there is a large sag that occurs because of loose pulleys and weight that will draw the intake down past when we cutoff motors. the intake should be considered extended at around -3 degrees.
    private static final double kAngleTolerance = 5;

    private currentStateIntakeRetractor currentState;

    public IntakeRetractorSubSystem(int intakeRetractorCANId, int intakeRetractorCANCoderId, double intakeRetractorCANCoderOffset) {

        // CANCoder config
        intakeRetractorAbsoluteEncoder = new CANcoder(intakeRetractorCANCoderId);

        CANcoderConfiguration CANCoderConfigIntakeRetractor = new CANcoderConfiguration();
        CANCoderConfigIntakeRetractor.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive; // clockwise???? counterclockwise???? -- they are facing the same direction so they should be the same
        CANCoderConfigIntakeRetractor.MagnetSensor.MagnetOffset = intakeRetractorCANCoderOffset;

        intakeRetractorAbsoluteEncoder.getConfigurator().apply(CANCoderConfigIntakeRetractor);

        // determine current position (set state)
        if ((kRetractedAngle + kAngleTolerance) >= getIntakeAngleDeg()) { // det. retracted
            currentState = currentStateIntakeRetractor.IDLE_RETRACTED;
            intakeRetractorMotorStopped = true;
        } else if (getIntakeAngleDeg() >= -kAngleTolerance) {
            currentState = currentStateIntakeRetractor.IDLE_EXTENDED;
            intakeRetractorMotorStopped = true;
        } else {
            currentState = currentStateIntakeRetractor.EXTENDING;
            intakeRetractorMotorStopped = true;
        }

        // Update alert to reflect initial state
        updateIntakeRetractorAlert();

        // SPARK MAX config
        intakeRetractorMotor = new SparkMax(intakeRetractorCANId, SparkMax.MotorType.kBrushless);

        sparkConfigIntakeRetractorMotor = new SparkMaxConfig();

        sparkConfigIntakeRetractorMotor
                .idleMode(IdleMode.kBrake)
                .inverted(false);
        sparkConfigIntakeRetractorMotor.encoder
                .positionConversionFactor(0.079365)
                .velocityConversionFactor(0.079365);
        sparkConfigIntakeRetractorMotor.smartCurrentLimit(60, 60);

        intakeRetractorMotor.configure(sparkConfigIntakeRetractorMotor, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);
    }

    private void updateIntakeRetractorAlert() {
        switch (currentState) {
            case IDLE_RETRACTED -> {
                intakeRetractorStatusAlert.setText("Intake Retracted");
                intakeRetractorStatusAlert.set(true);
            }
            case IDLE_EXTENDED -> {
                intakeRetractorStatusAlert.setText("Intake Extended");
                intakeRetractorStatusAlert.set(true);
            }
            case EXTENDING -> {
                intakeRetractorStatusAlert.setText("Intake EXTENDING");
                intakeRetractorStatusAlert.set(true);
            }
            default -> {
                intakeRetractorStatusAlert.set(false);
            }
        }
    }

    public Command doIntakeRetractionCmd() {
        return new InstantCommand(this::doIntakeRetraction, this);
    }

    public void endIntakeRetractionMotor() {
        intakeRetractorMotor.stopMotor();
    }

    public final double getIntakeAngleDeg() {
        return intakeRetractorAbsoluteEncoder.getPosition().getValueAsDouble() * 360.0;
    }

    public void doIntakeRetraction() {
        switch (currentState) {
            case IDLE_RETRACTED -> {
                currentState = currentStateIntakeRetractor.EXTENDING;
                intakeRetractorMotorStopped = false;
            }
            case IDLE_EXTENDED, EXTENDING -> {
                if (!intakeRetractionProhibitedRumbleActive && !DriverStation.isAutonomous()) {
                    intakeRetractionProhibitedRumbleTimer.reset();
                    intakeRetractionProhibitedRumbleTimer.start();
                    intakeRetractionProhibitedRumbleActive = true;
                }
            }
        }
    }

    public void intakeRetractorControl() {
        if (currentState == currentStateIntakeRetractor.EXTENDING) {
            if ((Math.abs(getIntakeAngleDeg() - kMotorCutoffAngleForExtension) > kAngleTolerance) && intakeRetractorMotorStopped == false) {
                    intakeRetractorMotor.setVoltage(IntakeRetractorConstants.IntakeRetractorVoltage);
                } else {
                    endIntakeRetractionMotor();
                    currentState = currentStateIntakeRetractor.IDLE_EXTENDED;
                    intakeRetractorMotorStopped = true;
                }
        } else if ((kRetractedAngle + kAngleTolerance) >= getIntakeAngleDeg()) {
            currentState = currentStateIntakeRetractor.IDLE_RETRACTED;
            intakeRetractorMotorStopped = true;
        }
    }  

    public void intakeRetractorPeriodic() {
        SmartDashboard.putNumber("Intake Retractor Angle", getIntakeAngleDeg());
        SmartDashboard.putString("CurrentState", currentState.toString());
        updateIntakeRetractorAlert();

        if (intakeRetractionProhibitedRumbleActive && !(DriverStation.getMatchTime() >= 28.0 && DriverStation.getMatchTime() <= 33.0)) {
            if (intakeRetractionProhibitedRumbleTimer.hasElapsed(1.0)) {
                RobotContainer.operatorController.setRumble(RumbleType.kBothRumble, 0.0);
                intakeRetractionProhibitedRumbleTimer.stop();
                intakeRetractionProhibitedRumbleActive = false;
            } else {
                RobotContainer.operatorController.setRumble(RumbleType.kBothRumble, 1.0);
            }
        }
    }
}
