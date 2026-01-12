package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubSystem extends SubsystemBase{
    SparkMax shooterMotor;
    SparkMaxConfig sparkConfigShooterMotor;

    public ShooterSubSystem(int shooterCanId) {
        shooterMotor = new SparkMax(shooterCanId, SparkMax.MotorType.kBrushless);

        sparkConfigShooterMotor = new SparkMaxConfig();

        sparkConfigShooterMotor
                .inverted(false);
        sparkConfigShooterMotor.encoder
                .positionConversionFactor(0.037037037 * Math.PI * 2)
                .velocityConversionFactor(0.037037037 * Math.PI * 2);
        sparkConfigShooterMotor.smartCurrentLimit(40, 40);
        
        // MAKE SURE TO UPDATE THE POSITION & VELOCITY CONVERSION FACTORS WHEN WE KNOW THE GEAR RATIOS
    }

    public void endShooterMotor() {
        shooterMotor.stopMotor();
    }

    public void shooterControl() {
        // default command
    }
}
