package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {

    private final SparkMax driveMotor;
    private final SparkMax turnMotor;

    private final SparkMaxConfig sparkConfigDrive;
    private final SparkMaxConfig sparkConfigTurn;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turnEncoder;

    private final AbsoluteEncoder driveAbsoluteEncoder;

    private final PIDController turnPidController;

    private final CANcoder absoluteEncoder;

    private final double chassisOffset = 0;

    public SwerveModule(int driveMotorId, int turnMotorId, boolean driveMotorReversed, boolean turnMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffset) {

        absoluteEncoder = new CANcoder(absoluteEncoderId);

        CANcoderConfiguration CANCoderConfig = new CANcoderConfiguration();
        CANCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        CANCoderConfig.MagnetSensor.MagnetOffset = absoluteEncoderOffset;

        absoluteEncoder.getConfigurator().apply(CANCoderConfig);

        driveMotor = new SparkMax(driveMotorId, SparkMax.MotorType.kBrushless);
        turnMotor = new SparkMax(turnMotorId, SparkMax.MotorType.kBrushless);
        sparkConfigDrive = new SparkMaxConfig();
        sparkConfigTurn = new SparkMaxConfig();

        sparkConfigDrive
                .idleMode(IdleMode.kBrake)
                .inverted(driveMotorReversed);
        sparkConfigDrive.encoder
                .positionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter)
                .velocityConversionFactor(ModuleConstants.kDriveEncoderRot2Meter / 60); // RPM to m/s
        sparkConfigDrive.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(1.0, 0.0, 0.0);
        sparkConfigDrive.smartCurrentLimit(60, 60);

        sparkConfigTurn
                .idleMode(IdleMode.kBrake)
                .inverted(turnMotorReversed);
        sparkConfigTurn.encoder
                .positionConversionFactor(ModuleConstants.kTurnEncoderRot2Rad)
                .velocityConversionFactor(ModuleConstants.kTurnEncoderRot2Rad / 60); // RPM to rad/s
        sparkConfigTurn.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(1.0, 0.0, 0.0);
        sparkConfigTurn.smartCurrentLimit(60, 60);

        driveMotor.configure(sparkConfigDrive, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        turnMotor.configure(sparkConfigTurn, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        driveEncoder = driveMotor.getEncoder();
        turnEncoder = turnMotor.getEncoder();

        driveAbsoluteEncoder = driveMotor.getAbsoluteEncoder();

        turnPidController = new PIDController(0.37, 0, 0);
        turnPidController.enableContinuousInput(-Math.PI, Math.PI);

        driveEncoder.setPosition(0);
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public double getTurningVelocity() {
        return turnEncoder.getVelocity();
    }

    public double getAbsoluteEncoderRad() {
        Rotation2d rot = Rotation2d.fromRadians((absoluteEncoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI));
        return rot.minus(Rotation2d.fromRadians(chassisOffset)).getRadians();
    }

    public void resetTurn() {
        double position = getAbsoluteEncoderRad();
        turnEncoder.setPosition(position);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getAbsoluteEncoderRad() - chassisOffset));
    }

    public void setDesiredState(SwerveModuleState state) {
        
        SwerveModuleState correctedSwerveModuleState = state;
        correctedSwerveModuleState.speedMetersPerSecond = state.speedMetersPerSecond;
        correctedSwerveModuleState.angle = state.angle.plus(Rotation2d.fromRadians(chassisOffset));
        
        if (Math.abs(correctedSwerveModuleState.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }

        correctedSwerveModuleState.optimize(getState().angle);
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turnMotor.set(turnPidController.calculate(getAbsoluteEncoderRad(), state.angle.getRadians()));
    }

    public void stop() {
        driveMotor.set(0);
        turnMotor.set(0);
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                driveAbsoluteEncoder.getPosition(),
                Rotation2d.fromRadians(getAbsoluteEncoderRad()));
    }

}