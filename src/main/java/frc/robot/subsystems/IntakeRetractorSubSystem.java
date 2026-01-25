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

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeRetractorConstants;

public class IntakeRetractorSubSystem extends SubsystemBase {

    SparkMax intakeRetractorLeftMotor;
    SparkMaxConfig sparkConfigIntakeRetractorLeftMotor;

    SparkMax intakeRetractorRightMotor;
    SparkMaxConfig sparkConfigIntakeRetractorRightMotor;

    private final CANcoder intakeRetractorAbsoluteEncoder;

    private final Timer intakeRetractionProhibitedRumbleTimer = new Timer();
    private boolean intakeRetractionProhibitedRumbleActive = false;

    boolean intakeRetractionMotorsStopped = true;

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

    public IntakeRetractorSubSystem(int intakeRetractorLeftCanId, int intakeRetractorRightCanId, int intakeRetractorCANCoderId, double intakeRetractorCANCoderOffset) {

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
            intakeRetractionMotorsStopped = true;
        } else if (Math.abs(angle - kExtendedAngle) <= kAngleTolerance) { // det. extended
            desiredDirection = desiredDirectionIntakeRetractor.EXTEND;
            currentState = currentStateIntakeRetractor.IDLE_EXTENDED;
            intakeRetractionMotorsStopped = true;
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
        intakeRetractorLeftMotor = new SparkMax(intakeRetractorLeftCanId, SparkMax.MotorType.kBrushless);
        intakeRetractorRightMotor = new SparkMax(intakeRetractorRightCanId, SparkMax.MotorType.kBrushless);

        sparkConfigIntakeRetractorLeftMotor = new SparkMaxConfig();
        sparkConfigIntakeRetractorRightMotor = new SparkMaxConfig();

        sparkConfigIntakeRetractorLeftMotor
                .idleMode(IdleMode.kBrake)
                .inverted(false);
        sparkConfigIntakeRetractorLeftMotor.encoder
                .positionConversionFactor(0.037037037 * Math.PI * 2)
                .velocityConversionFactor(0.037037037 * Math.PI * 2);
        sparkConfigIntakeRetractorLeftMotor.smartCurrentLimit(60, 60);

        sparkConfigIntakeRetractorRightMotor
                .idleMode(IdleMode.kBrake)
                .inverted(true);
        sparkConfigIntakeRetractorRightMotor.encoder
                .positionConversionFactor(0.037037037 * Math.PI * 2)
                .velocityConversionFactor(0.037037037 * Math.PI * 2);
        sparkConfigIntakeRetractorRightMotor.smartCurrentLimit(60, 60);
        // MAKE SURE TO UPDATE THE POSITION & VELOCITY CONVERSION FACTORS WHEN WE KNOW THE GEAR RATIOS

        intakeRetractorLeftMotor.configure(sparkConfigIntakeRetractorLeftMotor, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);
        intakeRetractorRightMotor.configure(sparkConfigIntakeRetractorRightMotor, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);
    }

    public void endIntakeRetractionMotors() {
        intakeRetractorLeftMotor.stopMotor();
        intakeRetractorRightMotor.stopMotor();
    }

    public double getIntakeAngleDeg() {
        return intakeRetractorAbsoluteEncoder.getPosition().getValueAsDouble() * 360.0;
    }

    public void doIntakeRetraction() {
        switch (currentState) {
            case IDLE_RETRACTED -> {desiredDirection = desiredDirectionIntakeRetractor.EXTEND; intakeRetractionMotorsStopped = false;}
            case IDLE_EXTENDED -> {desiredDirection = desiredDirectionIntakeRetractor.RETRACT; intakeRetractionMotorsStopped = false;}
            case EXTENDING, RETRACTING -> {
                if (!intakeRetractionProhibitedRumbleActive) {
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
                if (getIntakeAngleDeg() < kExtendedAngle - kAngleTolerance && intakeRetractionMotorsStopped == false) {
                    intakeRetractorLeftMotor.setVoltage(IntakeRetractorConstants.IntakeRetractorVoltage); // ASSUMES VOLTAGE IS POSITIVE TO EXTEND -- TEST
                    intakeRetractorRightMotor.setVoltage(IntakeRetractorConstants.IntakeRetractorVoltage);
                    currentState = currentStateIntakeRetractor.EXTENDING;
                } else { 
                    endIntakeRetractionMotors(); 
                    currentState = currentStateIntakeRetractor.IDLE_EXTENDED;
                    intakeRetractionMotorsStopped = true;
                }
            }

            case RETRACT -> {
                if (getIntakeAngleDeg() > kRetractedAngle + kAngleTolerance && intakeRetractionMotorsStopped == false) {
                    intakeRetractorLeftMotor.setVoltage(-IntakeRetractorConstants.IntakeRetractorVoltage); // ASSUMES VOLTAGE IS NEGATIVE TO RETRACT -- TEST
                    intakeRetractorRightMotor.setVoltage(-IntakeRetractorConstants.IntakeRetractorVoltage);
                    currentState = currentStateIntakeRetractor.RETRACTING;
                } else { 
                    endIntakeRetractionMotors(); 
                    currentState = currentStateIntakeRetractor.IDLE_RETRACTED;
                    intakeRetractionMotorsStopped = true;
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
