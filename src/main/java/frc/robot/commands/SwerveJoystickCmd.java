package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveSubSystem;
import frc.robot.subsystems.LimelightDetectionSubSystem;

public class SwerveJoystickCmd extends Command {

    private final SwerveSubSystem swerveSubsystem;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
    private final Supplier<Boolean> fieldOrientedFunction;
    private final Supplier<Boolean> aimAssistEnabledFunction;
    private final LimelightDetectionSubSystem limelightDetectionSubsystem;

    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

    /**
     * Creates a new SwerveJoystick.
     */
    public SwerveJoystickCmd(
            SwerveSubSystem swerveSubsystem,
            Supplier<Double> xSpdFunction,
            Supplier<Double> ySpdFunction,
            Supplier<Double> turningSpdFunction,
            Supplier<Boolean> fieldOrientedFunction,
            Supplier<Boolean> aimAssistEnabledFunction,
            LimelightDetectionSubSystem limelightDetectionSubsystem) {

        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.fieldOrientedFunction = fieldOrientedFunction;
        this.aimAssistEnabledFunction = aimAssistEnabledFunction;
        this.limelightDetectionSubsystem = limelightDetectionSubsystem;

        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        addRequirements(swerveSubsystem);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // 1. Get joystick inputs
        double xSpeed = xSpdFunction.get();
        double ySpeed = ySpdFunction.get();
        double turningSpeed = turningSpdFunction.get();

        double xSpeedBeforeLimelight;
        double ySpeedBeforeLimelight;
        double turningSpeedBeforeLimelight;

        // 2. Apply deadband
        xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;

        // 3. Make the driving smoother
        if (RobotContainer.driverController.getLeftBumperButton()) { // if LB button not pressed, robot moves at full speed. while LB held, driving is slower & more precise
            xSpeed = xLimiter.calculate(xSpeed) * (DriveConstants.kTeleDriveMaxSpeedMetersPerSecond * DriveConstants.kSlowButtonDriveModifier);
            ySpeed = yLimiter.calculate(ySpeed) * (DriveConstants.kTeleDriveMaxSpeedMetersPerSecond * DriveConstants.kSlowButtonDriveModifier);
            turningSpeed = turningLimiter.calculate(turningSpeed) * (DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond * DriveConstants.kSlowButtonTurnModifier);
        } else {
            xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
            ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
            turningSpeed = turningLimiter.calculate(turningSpeed) * (DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond);
        }

        // 3.5 If aim assist is enabled, override joystick inputs with Limelight corrections (blended with driver input)
        if (aimAssistEnabledFunction.get()) {
            xSpeedBeforeLimelight = xSpeed;
            ySpeedBeforeLimelight = ySpeed;
            turningSpeedBeforeLimelight = turningSpeed;
            xSpeed += limelightDetectionSubsystem.getXSpeedLimelight();
            ySpeed += limelightDetectionSubsystem.getYSpeedLimelight();
            turningSpeed += limelightDetectionSubsystem.getTurnSpeedLimelight();
        } else {
            xSpeedBeforeLimelight = xSpeed;
            ySpeedBeforeLimelight = ySpeed;
            turningSpeedBeforeLimelight = turningSpeed;
        }

        // 3.75 driver rumble when attempting to drive against active limelight corrections
        if (aimAssistEnabledFunction.get()) {
            if (DriverStation.isFMSAttached()) {
                if (!(DriverStation.getMatchTime() >= 28.0 && DriverStation.getMatchTime() <= 33.0)) {
                    if ((limelightDetectionSubsystem.getXSpeedLimelight() != 0 && xSpeedBeforeLimelight != 0) || (limelightDetectionSubsystem.getYSpeedLimelight() != 0 && ySpeedBeforeLimelight != 0) || (limelightDetectionSubsystem.getTurnSpeedLimelight() != 0 && turningSpeedBeforeLimelight != 0)) {
                        RobotContainer.driverController.setRumble(RumbleType.kBothRumble, 1.0);
                    } else {
                        RobotContainer.driverController.setRumble(RumbleType.kBothRumble, 0.0);
                    }
                }
            } else {
                if ((limelightDetectionSubsystem.getXSpeedLimelight() != 0 && xSpeedBeforeLimelight != 0) || (limelightDetectionSubsystem.getYSpeedLimelight() != 0 && ySpeedBeforeLimelight != 0) || (limelightDetectionSubsystem.getTurnSpeedLimelight() != 0 && turningSpeedBeforeLimelight != 0)) {
                    RobotContainer.driverController.setRumble(RumbleType.kBothRumble, 1.0);
                } else {
                    RobotContainer.driverController.setRumble(RumbleType.kBothRumble, 0.0);
                }
            }
        } else if (!(DriverStation.getMatchTime() >= 28.0 && DriverStation.getMatchTime() <= 33.0)) {
            RobotContainer.driverController.setRumble(RumbleType.kBothRumble, 0.0);
        }

        // 4. Construct desired chassis speeds
        ChassisSpeeds chassisSpeeds;
        if (fieldOrientedFunction.get()) {
            if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
                chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(-xSpeed, -ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
            } else {
                chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
            }
        } else {
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        }

        // 5. Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        // 6. Output each module states to the wheels
        swerveSubsystem.setModuleStates(moduleStates);

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
