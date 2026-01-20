package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubSystem extends SubsystemBase {

    SparkMax climberLeftMotor;
    SparkMaxConfig sparkConfigClimberLeftMotor;

    SparkMax climberRightMotor;
    SparkMaxConfig sparkConfigClimberRightMotor;

    public ClimberSubSystem(int climberLeftCanId, int climberRightCanId) {
        climberLeftMotor = new SparkMax(climberLeftCanId, SparkMax.MotorType.kBrushless);
        climberRightMotor = new SparkMax(climberRightCanId, SparkMax.MotorType.kBrushless);

        sparkConfigClimberLeftMotor = new SparkMaxConfig();
        sparkConfigClimberRightMotor = new SparkMaxConfig();

        sparkConfigClimberLeftMotor
                .idleMode(IdleMode.kBrake)
                .inverted(false);
        sparkConfigClimberLeftMotor.encoder
                .positionConversionFactor(0.037037037 * Math.PI * 2)
                .velocityConversionFactor(0.037037037 * Math.PI * 2);
        sparkConfigClimberLeftMotor.smartCurrentLimit(60, 60);

        sparkConfigClimberRightMotor
                .idleMode(IdleMode.kBrake)
                .inverted(true);
        sparkConfigClimberRightMotor.encoder
                .positionConversionFactor(0.037037037 * Math.PI * 2)
                .velocityConversionFactor(0.037037037 * Math.PI * 2);
        sparkConfigClimberRightMotor.smartCurrentLimit(60, 60);

        // MAKE SURE TO UPDATE THE POSITION & VELOCITY CONVERSION FACTORS WHEN WE KNOW THE GEAR RATIOS
    }

    public void endClimberMotors() {
        climberLeftMotor.stopMotor();
        climberRightMotor.stopMotor();
    }

    public void climberControl() {
        // default command
    }

}
