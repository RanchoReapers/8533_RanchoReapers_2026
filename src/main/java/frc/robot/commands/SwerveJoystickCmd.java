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
    private final Supplier<Double> driverXSpdFunction, driverYSpdFunction, driverTurningSpdFunction;
    private final Supplier<Double> operatorXSpdFunction, operatorYSpdFunction, operatorTurningSpdFunction;
    private final Supplier<Boolean> fieldOrientedFunction;
    private final Supplier<Boolean> aimAssistEnabledFunction;
    public final LimelightDetectionSubSystem limelightDetectionSubsystem;

    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

    /**
     * Creates a new SwerveJoystick.
     */
    public SwerveJoystickCmd(
            SwerveSubSystem swerveSubsystem,
            Supplier<Double> driverXSpdFunction,
            Supplier<Double> driverYSpdFunction,
            Supplier<Double> driverTurningSpdFunction,
            Supplier<Double> operatorXSpdFunction,
            Supplier<Double> operatorYSpdFunction,
            Supplier<Double> operatorTurningSpdFunction,
            Supplier<Boolean> fieldOrientedFunction,
            Supplier<Boolean> aimAssistEnabledFunction,
            LimelightDetectionSubSystem limelightDetectionSubsystem) {

        this.swerveSubsystem = swerveSubsystem;
        this.driverXSpdFunction = driverXSpdFunction;
        this.driverYSpdFunction = driverYSpdFunction;
        this.driverTurningSpdFunction = driverTurningSpdFunction;
        this.operatorXSpdFunction = operatorXSpdFunction;
        this.operatorYSpdFunction = operatorYSpdFunction;
        this.operatorTurningSpdFunction = operatorTurningSpdFunction;
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
        if (RobotContainer.operatorController.getLeftBumperButton()) { // operator controller driving override on left bumper button -> constraints: must drive robot oriented, aim assist disabled, slow mode is on right bumper
            
            // 1. Get joystick inputs
            double xSpeed = operatorXSpdFunction.get();
            double ySpeed = operatorYSpdFunction.get();
            double turningSpeed = operatorTurningSpdFunction.get();

            // 2. Apply deadband
            xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
            ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
            turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;

            // 3. Make the driving smoother, and apply slow mode if operator right bumper held
            if (RobotContainer.operatorController.getRightBumperButton()) { // if RB button not pressed, robot moves at full speed. while RB held, driving is slower & more precise
                xSpeed = xLimiter.calculate(xSpeed) * (DriveConstants.kTeleDriveMaxSpeedMetersPerSecond * DriveConstants.kSlowButtonDriveModifier);
                ySpeed = yLimiter.calculate(ySpeed) * (DriveConstants.kTeleDriveMaxSpeedMetersPerSecond * DriveConstants.kSlowButtonDriveModifier);
                turningSpeed = turningLimiter.calculate(turningSpeed) * (DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond * DriveConstants.kSlowButtonTurnModifier);
            } else {
                xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
                ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
                turningSpeed = turningLimiter.calculate(turningSpeed) * (DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond);
            }

            // 3.5 - 3.75. Aim assist corrections disabled when operator driving override active

            // 4. Construct desired chassis speeds, field oriented control disabled.
            ChassisSpeeds chassisSpeeds;
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);

            // 5. Convert chassis speeds to individual module states
            SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

            // 6. Output each module states to the wheels
            swerveSubsystem.setModuleStates(moduleStates);

        } else {

            // 1. Get joystick inputs
            double xSpeed = driverXSpdFunction.get();
            double ySpeed = driverYSpdFunction.get();
            double turningSpeed = driverTurningSpdFunction.get();

            // 2. Apply deadband
            xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
            ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
            turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;

             // 3. Make the driving smoother, and apply slow mode if driver left bumper held
            if (RobotContainer.driverController.getLeftBumperButton()) { // if LB button not pressed, robot moves at full speed. while LB held, driving is slower & more precise
                xSpeed = xLimiter.calculate(xSpeed) * (DriveConstants.kTeleDriveMaxSpeedMetersPerSecond * DriveConstants.kSlowButtonDriveModifier);
                ySpeed = yLimiter.calculate(ySpeed) * (DriveConstants.kTeleDriveMaxSpeedMetersPerSecond * DriveConstants.kSlowButtonDriveModifier);
                turningSpeed = turningLimiter.calculate(turningSpeed) * (DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond * DriveConstants.kSlowButtonTurnModifier);
            } else {
                xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
                ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
                turningSpeed = turningLimiter.calculate(turningSpeed) * (DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond);
            }

            // 3.5. If aim assist is enabled, override joystick inputs with Limelight corrections (blended with driver input). Respects slow drive/turn modifiers.
            limelightDetectionSubsystem.xSpeedBeforeLimelight = xSpeed;
            limelightDetectionSubsystem.ySpeedBeforeLimelight = ySpeed;
            limelightDetectionSubsystem.turningSpeedBeforeLimelight = turningSpeed;

            if (aimAssistEnabledFunction.get() && RobotContainer.driverController.getLeftBumperButton()) {
                xSpeed += (limelightDetectionSubsystem.getXSpeedLimelight() * DriveConstants.kSlowButtonDriveModifier);
                ySpeed += (limelightDetectionSubsystem.getYSpeedLimelight() * DriveConstants.kSlowButtonDriveModifier);
                turningSpeed += (limelightDetectionSubsystem.getTurnSpeedLimelight() * DriveConstants.kSlowButtonTurnModifier);
            } else if (aimAssistEnabledFunction.get()) {
                xSpeed += limelightDetectionSubsystem.getXSpeedLimelight();
                ySpeed += limelightDetectionSubsystem.getYSpeedLimelight();
                turningSpeed += limelightDetectionSubsystem.getTurnSpeedLimelight();
            }

            // 3.75. driver rumble when attempting to drive against active limelight corrections
            if (aimAssistEnabledFunction.get()) {
                if (DriverStation.isFMSAttached()) {
                    if (!(DriverStation.getMatchTime() >= 28.0 && DriverStation.getMatchTime() <= 33.0)) {
                        if ((limelightDetectionSubsystem.getXSpeedLimelight() != 0 && limelightDetectionSubsystem.xSpeedBeforeLimelight != 0) || (limelightDetectionSubsystem.getYSpeedLimelight() != 0 && limelightDetectionSubsystem.ySpeedBeforeLimelight != 0) || (limelightDetectionSubsystem.getTurnSpeedLimelight() != 0 && limelightDetectionSubsystem.turningSpeedBeforeLimelight != 0)) {
                            RobotContainer.driverController.setRumble(RumbleType.kBothRumble, 1.0);
                        } else {
                            RobotContainer.driverController.setRumble(RumbleType.kBothRumble, 0.0);
                        }
                    }
                } else {
                    if ((limelightDetectionSubsystem.getXSpeedLimelight() != 0 && limelightDetectionSubsystem.xSpeedBeforeLimelight != 0) || (limelightDetectionSubsystem.getYSpeedLimelight() != 0 && limelightDetectionSubsystem.ySpeedBeforeLimelight != 0) || (limelightDetectionSubsystem.getTurnSpeedLimelight() != 0 && limelightDetectionSubsystem.turningSpeedBeforeLimelight != 0)) {
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
