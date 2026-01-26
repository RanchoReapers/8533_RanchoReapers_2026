package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeRetractorConstants;
import edu.wpi.first.wpilibj.DriverStation;

public class IntakeRetractorSubSystem extends SubsystemBase {

    SparkMax intakeRetractorMotor;
    SparkMaxConfig sparkConfigIntakeRetractorMotor;

    private final CANcoder intakeRetractorAbsoluteEncoder;

    private final Timer intakeRetractionProhibitedRumbleTimer = new Timer();
    private boolean intakeRetractionProhibitedRumbleActive = false;

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

    private static final double kRetractedAngle = 0.0;
    private static final double kExtendedAngle = 90.0;
    private static final double kAngleTolerance = 2.0;
    // change the angles once you calibrate CANCoders

    private currentStateIntakeRetractor currentState;
    private desiredDirectionIntakeRetractor desiredDirection;

    public IntakeRetractorSubSystem(int intakeRetractorCanId, int intakeRetractorCANCoderId, double intakeRetractorCANCoderOffset) {

        // CANCoder config
        intakeRetractorAbsoluteEncoder = new CANcoder(intakeRetractorCANCoderId);

        CANcoderConfiguration CANCoderConfigIntakeRetractor = new CANcoderConfiguration();

        CANCoderConfigIntakeRetractor.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive; // clockwise???? counterclockwise???? -- they are facing the same direction so they should be the same
        CANCoderConfigIntakeRetractor.MagnetSensor.MagnetOffset = intakeRetractorCANCoderOffset;

        intakeRetractorAbsoluteEncoder.getConfigurator().apply(CANCoderConfigIntakeRetractor);

        double angle = getIntakeAngleDeg();

        // determine current position (set state)
        if (Math.abs(angle - kRetractedAngle) <= kAngleTolerance) { // det. retracted
            desiredDirection = desiredDirectionIntakeRetractor.RETRACT;
            currentState = currentStateIntakeRetractor.IDLE_RETRACTED;
            intakeRetractionMotorStopped = true;
        } else if (Math.abs(angle - kExtendedAngle) <= kAngleTolerance) { // det. extended
            desiredDirection = desiredDirectionIntakeRetractor.EXTEND;
            currentState = currentStateIntakeRetractor.IDLE_EXTENDED;
            intakeRetractionMotorStopped = true;
        } else {
            double distToRetracted = Math.abs(angle - kRetractedAngle);
            double distToExtended = Math.abs(angle - kExtendedAngle);
            if (distToRetracted <= distToExtended) {
                desiredDirection = desiredDirectionIntakeRetractor.RETRACT;
                currentState = currentStateIntakeRetractor.RETRACTING; // closer to retracted or tie => assume retracting
            } else {
                desiredDirection = desiredDirectionIntakeRetractor.EXTEND;
                currentState = currentStateIntakeRetractor.EXTENDING; // closer to extended => assume extending
            }
        }

        // SPARK MAX config
        intakeRetractorMotor = new SparkMax(intakeRetractorCanId, SparkMax.MotorType.kBrushless);

        sparkConfigIntakeRetractorMotor = new SparkMaxConfig();

        sparkConfigIntakeRetractorMotor
                .idleMode(IdleMode.kBrake)
                .inverted(false);
        sparkConfigIntakeRetractorMotor.encoder
                .positionConversionFactor(0.037037037 * Math.PI * 2)
                .velocityConversionFactor(0.037037037 * Math.PI * 2);
        sparkConfigIntakeRetractorMotor.smartCurrentLimit(60, 60);
        // MAKE SURE TO UPDATE THE POSITION & VELOCITY CONVERSION FACTORS WHEN WE KNOW THE GEAR RATIOS

        intakeRetractorMotor.configure(sparkConfigIntakeRetractorMotor, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);
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

    public void intakeRetractorControl() {
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
