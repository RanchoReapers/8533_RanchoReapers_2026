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

    boolean intakeRetractorDirectionIsRetract;
    boolean intakeRetractionMotorsStopped = true;
    boolean isInMotion;

    public IntakeRetractorSubSystem(int intakeRetractorLeftCanId, int intakeRetractorRightCanId,
            int intakeRetractorCANCoderId, double intakeRetractorCANCoderOffset) {

        // CANCoder config
        intakeRetractorAbsoluteEncoder = new CANcoder(intakeRetractorCANCoderId);

        CANcoderConfiguration CANCoderConfigIntakeRetractor = new CANcoderConfiguration();

        CANCoderConfigIntakeRetractor.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive; // clockwise???? counterclockwise???? -- they are facing the same direction so they should be the same
        CANCoderConfigIntakeRetractor.MagnetSensor.MagnetOffset = intakeRetractorCANCoderOffset;

        intakeRetractorAbsoluteEncoder.getConfigurator().apply(CANCoderConfigIntakeRetractor);

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
    }

    public void endIntakeRetractionMotors() {
        intakeRetractorLeftMotor.stopMotor();
        intakeRetractorRightMotor.stopMotor();
    }

    public double getIntakeAngleDeg() {
        double angle = intakeRetractorAbsoluteEncoder.getPosition().getValueAsDouble() * 360.0;
        angle = Math.IEEEremainder(angle, 360.0);
        if (angle < 0) {
            angle += 360.0;
        }
        return angle;
    }

    public void doIntakeRetraction() {
        if (isInMotion == false && getIntakeAngleDeg() <= 1 && getIntakeAngleDeg() >= -1) { // 0 degrees is retracted w/ a 1 degree tolerance
            intakeRetractorDirectionIsRetract = false;
            intakeRetractionMotorsStopped = false;
            isInMotion = true;
        } else if (isInMotion == false && getIntakeAngleDeg() <= 91 && getIntakeAngleDeg() >= 89) { // CHANGE 90 to whatever the extended angle is -- 90 degrees is extended w/ a 1 degree tolerance
            intakeRetractorDirectionIsRetract = true;
            intakeRetractionMotorsStopped = false;
            isInMotion = true;
        } else {
            if (!intakeRetractionProhibitedRumbleActive) {
                intakeRetractionProhibitedRumbleTimer.reset();
                intakeRetractionProhibitedRumbleTimer.start();
                intakeRetractionProhibitedRumbleActive = true;
            }
        }
    }

    public void intakeRetractorControl() {
        // 0 degrees is RETRACTED
        if (intakeRetractionMotorsStopped == false && intakeRetractorDirectionIsRetract == true) {
            if (getIntakeAngleDeg() > 1) { // 1 degree tolerance (0)
                intakeRetractorLeftMotor.setVoltage(-IntakeRetractorConstants.IntakeRetractorVoltage); // ASSUMES VOLTAGE IS NEGATIVE TO RETRACT -- TEST
                intakeRetractorRightMotor.setVoltage(-IntakeRetractorConstants.IntakeRetractorVoltage);
                isInMotion = true;
            } else {
                endIntakeRetractionMotors();
                isInMotion = false;
                intakeRetractionMotorsStopped = true;
            }
        } else if (intakeRetractionMotorsStopped == false && intakeRetractorDirectionIsRetract == false) {
            if (getIntakeAngleDeg() < 89) { // 1 degree tolerance (90)
                intakeRetractorLeftMotor.setVoltage(IntakeRetractorConstants.IntakeRetractorVoltage);
                intakeRetractorRightMotor.setVoltage(IntakeRetractorConstants.IntakeRetractorVoltage);
                isInMotion = true;
            } else {
                endIntakeRetractionMotors();
                isInMotion = false;
                intakeRetractionMotorsStopped = true;
            }
        } else {
            endIntakeRetractionMotors();
            isInMotion = false;
            intakeRetractionMotorsStopped = true;
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
