package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeRetractorConstants;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

public class IntakeRetractorSubSystem extends SubsystemBase {

    SparkMax intakeRetractorMotor;
    SparkMaxConfig sparkConfigIntakeRetractorMotor;

    private final CANcoder intakeRetractorAbsoluteEncoder;

    private final Timer intakeRetractionProhibitedRumbleTimer = new Timer();
    private boolean intakeRetractionProhibitedRumbleActive = false;

    // Info alert to indicate when the intake is retracted or extended
    private final Alert intakeRetractorStatusAlert = new Alert("Intake Retractor status", AlertType.kInfo);

    boolean intakeRetractionMotorStopped = true;

    public enum desiredDirectionIntakeRetractor {
        RETRACT,
        EXTEND,
    }

    public enum currentStateIntakeRetractor {
        IDLE_RETRACTED,
        IDLE_EXTENDED,
        EXTENDING,
        RETRACTING
    }

    private static final double kRetractedAngle = 85.0;
    private static final double kExtendedAngle = 0.0;
    private static final double kAngleTolerance = 5;

    private static final double kHoldVoltsPerDeg = 0.02;
    private static final double kMaxHoldVoltage = 3.0;
    // change the angles once you calibrate CANCoders

    private currentStateIntakeRetractor currentState;
    private desiredDirectionIntakeRetractor desiredDirection;
    
    public IntakeRetractorSubSystem(int intakeRetractorCanId, int intakeRetractorCANCoderId, double intakeRetractorCANCoderOffset) {

        // CANCoder config
        intakeRetractorAbsoluteEncoder = new CANcoder(intakeRetractorCANCoderId);

        CANcoderConfiguration CANCoderConfigIntakeRetractor = new CANcoderConfiguration();
        CANCoderConfigIntakeRetractor.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        CANCoderConfigIntakeRetractor.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive; // clockwise???? counterclockwise???? -- they are facing the same direction so they should be the same
        CANCoderConfigIntakeRetractor.MagnetSensor.MagnetOffset = intakeRetractorCANCoderOffset;

        intakeRetractorAbsoluteEncoder.getConfigurator().apply(CANCoderConfigIntakeRetractor);

        // determine current position (set state)
        if (Math.abs(getIntakeAngleDeg() - kRetractedAngle) <= kAngleTolerance) { // det. retracted
            desiredDirection = desiredDirectionIntakeRetractor.RETRACT;
            currentState = currentStateIntakeRetractor.IDLE_RETRACTED;
            intakeRetractionMotorStopped = true;
        } else if (Math.abs(getIntakeAngleDeg() - kExtendedAngle) <= kAngleTolerance) { // det. extended
            desiredDirection = desiredDirectionIntakeRetractor.EXTEND;
            currentState = currentStateIntakeRetractor.IDLE_EXTENDED;
            intakeRetractionMotorStopped = true;
        } else {
            double distToRetracted = Math.abs(getIntakeAngleDeg() - kRetractedAngle);
            double distToExtended = Math.abs(getIntakeAngleDeg() - kExtendedAngle);
            if (distToRetracted <= distToExtended) {
                desiredDirection = desiredDirectionIntakeRetractor.RETRACT;
                currentState = currentStateIntakeRetractor.RETRACTING; // closer to retracted or tie => assume retracting
            } else {
                desiredDirection = desiredDirectionIntakeRetractor.EXTEND;
                currentState = currentStateIntakeRetractor.EXTENDING; // closer to extended => assume extending
            }
        }

        // Update alert to reflect initial state
        updateIntakeRetractorAlert();

        // SPARK MAX config
        intakeRetractorMotor = new SparkMax(intakeRetractorCanId, SparkMax.MotorType.kBrushless);

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
            case RETRACTING -> {
                intakeRetractorStatusAlert.setText("Intake RETRACTING");
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

    public double getIntakeAngleDeg() {
        return intakeRetractorAbsoluteEncoder.getPosition().getValueAsDouble() * 360.0;
    }

    public void doIntakeRetraction() {
        switch (currentState) {
            case IDLE_RETRACTED -> {
                desiredDirection = desiredDirectionIntakeRetractor.EXTEND;
                intakeRetractionMotorStopped = false;
            }
            case IDLE_EXTENDED -> {
                desiredDirection = desiredDirectionIntakeRetractor.RETRACT;
                intakeRetractionMotorStopped = false;
            }
            case EXTENDING, RETRACTING -> {
                if (!intakeRetractionProhibitedRumbleActive && !DriverStation.isAutonomous()) {
                    intakeRetractionProhibitedRumbleTimer.reset();
                    intakeRetractionProhibitedRumbleTimer.start();
                    intakeRetractionProhibitedRumbleActive = true;
                }
            }
        }
    }

    private double clamp(double v, double lo, double hi) {
        if (v < lo) return lo;
        if (v > hi) return hi;
        return v;
    }

    public void intakeRetractorControl() {
        
        /*
        if (currentState == currentStateIntakeRetractor.IDLE_EXTENDED && intakeRetractionMotorStopped && (desiredDirection != desiredDirectionIntakeRetractor.RETRACT)) {
            double error = getIntakeAngleDeg() - kExtendedAngle;

            if (Math.abs(error) > kAngleTolerance) {

                double holdVoltage = clamp(-kHoldVoltsPerDeg * error, -kMaxHoldVoltage, kMaxHoldVoltage);
                if (Math.abs(holdVoltage) > 0.01) {
                    intakeRetractorMotor.setVoltage(holdVoltage);
                } else {
                    endIntakeRetractionMotor();
                    intakeRetractionMotorStopped = true;
                }
            } else {
                endIntakeRetractionMotor();
                intakeRetractionMotorStopped = true;
            }
            return; // don't run the movement switch while holding
        }
        */
        switch (desiredDirection) {

            case EXTEND -> {
                if (getIntakeAngleDeg() < kExtendedAngle - kAngleTolerance && intakeRetractionMotorStopped == false) {
                    intakeRetractorMotor.setVoltage(IntakeRetractorConstants.IntakeRetractorVoltage); // ASSUMES VOLTAGE IS POSITIVE TO EXTEND -- TEST
                    currentState = currentStateIntakeRetractor.EXTENDING;
                } else {
                    endIntakeRetractionMotor();
                    currentState = currentStateIntakeRetractor.IDLE_EXTENDED;
                    intakeRetractionMotorStopped = true;
                }
            }

            case RETRACT -> {
                if (getIntakeAngleDeg() > kRetractedAngle + kAngleTolerance && intakeRetractionMotorStopped == false) {
                    intakeRetractorMotor.setVoltage(-IntakeRetractorConstants.IntakeRetractorVoltage); // ASSUMES VOLTAGE IS NEGATIVE TO RETRACT -- TEST
                    currentState = currentStateIntakeRetractor.RETRACTING;
                } else {
                    endIntakeRetractionMotor();
                    currentState = currentStateIntakeRetractor.IDLE_RETRACTED;
                    intakeRetractionMotorStopped = true;
                }
            }
        }
    }

    public void intakeRetractorPeriodic() {
        SmartDashboard.putNumber("Intake Retractor Angle", getIntakeAngleDeg());
        updateIntakeRetractorAlert();
        if (intakeRetractionProhibitedRumbleActive) {
            RobotContainer.operatorController.setRumble(RumbleType.kBothRumble, 1.0);
            if (intakeRetractionProhibitedRumbleTimer.hasElapsed(1.0)) {
                RobotContainer.operatorController.setRumble(RumbleType.kBothRumble, 0.0);
                intakeRetractionProhibitedRumbleTimer.stop();
                intakeRetractionProhibitedRumbleActive = false;
            }
        }
    }
}
