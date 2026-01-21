package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeRetractorConstants;

public class IntakeRetractorSubSystem extends SubsystemBase {

    SparkMax intakeRetractorLeftMotor;
    SparkMaxConfig sparkConfigIntakeRetractorLeftMotor;

    SparkMax intakeRetractorRightMotor;
    SparkMaxConfig sparkConfigIntakeRetractorRightMotor;

    private final CANcoder intakeRetractorAbsoluteEncoder;

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
        return intakeRetractorAbsoluteEncoder.getPosition().getValueAsDouble() * 360.0;
    }

    public void doIntakeRetraction() {
        if (isInMotion == false && getIntakeAngleDeg() == 0) {
        intakeRetractorDirectionIsRetract = false;
        intakeRetractionMotorsStopped = false;
        } else if (isInMotion == false && getIntakeAngleDeg() == 90) { // CHANGE 90 to whatever the extended angle is
        intakeRetractorDirectionIsRetract = true;
        intakeRetractionMotorsStopped = false;
        }
    }

    public void intakeRetractorControl() {
        // 0 degrees is RETRACTED
        if (intakeRetractionMotorsStopped == false && intakeRetractorDirectionIsRetract == true) {
            intakeRetractorLeftMotor.setVoltage(-IntakeRetractorConstants.IntakeRetractorVoltage); // ASSUMES VOLTAGE IS NEGATIVE TO RETRACT -- TEST
            intakeRetractorRightMotor.setVoltage(-IntakeRetractorConstants.IntakeRetractorVoltage);
            isInMotion = true;
        } else if (intakeRetractionMotorsStopped == false && intakeRetractorDirectionIsRetract == false) { 
            intakeRetractorLeftMotor.setVoltage(IntakeRetractorConstants.IntakeRetractorVoltage); 
            intakeRetractorRightMotor.setVoltage(IntakeRetractorConstants.IntakeRetractorVoltage);
            isInMotion = true;
        } else {
            endIntakeRetractionMotors();
            isInMotion = false;
            intakeRetractionMotorsStopped = true;
        }
    }

    public void intakeRetractorPeriodic() {
        
    }

}
