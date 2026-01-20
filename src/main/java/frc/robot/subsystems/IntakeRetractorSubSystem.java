package frc.robot.subsystems;

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

    public IntakeRetractorSubSystem(int intakeRetractorLeftCanId, int intakeRetractorRightCanId) {
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

    public void endIntakeRetractorMotors() {
        intakeRetractorLeftMotor.stopMotor();
        intakeRetractorRightMotor.stopMotor();
    }

    public void intakeRetractorControl() {
        // default command -- MOTORS FACE THE SAME DIRECTION -- REQUIRES CANCoders
    }

    public void intakeRetractorPeriodic() {
    }

}
