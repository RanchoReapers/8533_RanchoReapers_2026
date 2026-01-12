package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubSystem extends SubsystemBase {

    SparkMax elevatorLeftMotor;
    SparkMaxConfig sparkConfigElevatorLeftMotor;

    SparkMax elevatorRightMotor;
    SparkMaxConfig sparkConfigElevatorRightMotor;

    public ClimberSubSystem(int elevatorLeftCanId, int elevatorRightCanId) {
        elevatorLeftMotor = new SparkMax(elevatorLeftCanId, SparkMax.MotorType.kBrushless);
        elevatorRightMotor = new SparkMax(elevatorRightCanId, SparkMax.MotorType.kBrushless);

        sparkConfigElevatorLeftMotor = new SparkMaxConfig();
        sparkConfigElevatorRightMotor = new SparkMaxConfig();

        sparkConfigElevatorLeftMotor
                .idleMode(IdleMode.kBrake)
                .inverted(false);
        sparkConfigElevatorLeftMotor.encoder
                .positionConversionFactor(0.037037037 * Math.PI * 2)
                .velocityConversionFactor(0.037037037 * Math.PI * 2);
        sparkConfigElevatorLeftMotor.smartCurrentLimit(60, 60);

        sparkConfigElevatorRightMotor
                .idleMode(IdleMode.kBrake)
                .inverted(true);
        sparkConfigElevatorRightMotor.encoder
                .positionConversionFactor(0.037037037 * Math.PI * 2)
                .velocityConversionFactor(0.037037037 * Math.PI * 2);
        sparkConfigElevatorRightMotor.smartCurrentLimit(60, 60);

        // MAKE SURE TO UPDATE THE POSITION & VELOCITY CONVERSION FACTORS WHEN WE KNOW THE GEAR RATIOS
    }

    public void endElevatorMotors() {
        elevatorLeftMotor.stopMotor();
        elevatorRightMotor.stopMotor();
    }

    public void elevatorControl() {
        // default command
    }

}
