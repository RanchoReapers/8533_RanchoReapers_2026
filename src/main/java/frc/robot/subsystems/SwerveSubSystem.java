package frc.robot.subsystems;

import com.studica.frc.Navx; // ignore errors on this line

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.AutoConstants;

import choreo.trajectory.SwerveSample;
import choreo.auto.AutoFactory;

import choreo.Choreo.TrajectoryLogger;
import edu.wpi.first.math.controller.PIDController;

public class SwerveSubSystem extends SubsystemBase {

    private final SwerveModule frontLeft = new SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftTurningEncoderReversed,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad);

    private final SwerveModule frontRight = new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed,
            DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
            DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad);

    private final SwerveModule backLeft = new SwerveModule(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveEncoderReversed,
            DriveConstants.kBackLeftTurningEncoderReversed,
            DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
            DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad);

    private final SwerveModule backRight = new SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightTurningEncoderReversed,
            DriveConstants.kBackRightDriveAbsoluteEncoderPort,
            DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad);

    private final Navx gyro = new Navx(DriveConstants.navXCANID); // ignore errors on this line

    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(
            DriveConstants.kDriveKinematics, new Rotation2d(),
            new SwerveModulePosition[]{
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
            }, new Pose2d(5.0, 13.5, new Rotation2d()));

    private final PIDController pathXController = new PIDController(AutoConstants.kPXController, 0.0, 0.0);
    private final PIDController pathYController = new PIDController(AutoConstants.kPYController, 0.0, 0.0);
    private final PIDController pathThetaController = new PIDController(AutoConstants.kPThetaController, 0.0, 0.0);
    // educated guesses, tune ^

    // SHOULD LIMIT ACCEL FOR AUTO PATH FOLLOWING
    private double lastVx = 0.0;
    private double lastVy = 0.0;
    private double lastOmega = 0.0;
    private static final double CONTROL_LOOP_DT = 0.02; // time in seconds

    
    // auto path following \/
    public void followPath(SwerveSample sample) {
        pathThetaController.enableContinuousInput(-Math.PI, Math.PI);

        Pose2d pose = getPose();  // current robot pose

        ChassisSpeeds speeds = sample.getChassisSpeeds();

        // PID corrections
        speeds.vxMetersPerSecond += pathXController.calculate(pose.getX(), sample.x);
        speeds.vyMetersPerSecond += pathYController.calculate(pose.getY(), sample.y);
        speeds.omegaRadiansPerSecond += pathThetaController.calculate(
                pose.getRotation().getRadians(),
                sample.heading
        );

        // -- Accel limiting--
        double maxDeltaV = AutoConstants.kMaxAccelerationMetersPerSecondSquared * CONTROL_LOOP_DT;
        double maxDeltaOmega = AutoConstants.kMaxAngularAccelerationRadiansPerSecondSquared * CONTROL_LOOP_DT;

        // limit change in vx
        double deltaVx = speeds.vxMetersPerSecond - lastVx;
        if (Math.abs(deltaVx) > maxDeltaV) {
            speeds.vxMetersPerSecond = lastVx + Math.copySign(maxDeltaV, deltaVx);
        }
        // limit change in vy
        double deltaVy = speeds.vyMetersPerSecond - lastVy;
        if (Math.abs(deltaVy) > maxDeltaV) {
            speeds.vyMetersPerSecond = lastVy + Math.copySign(maxDeltaV, deltaVy);
        }
        // limit change in omega
        double deltaOmega = speeds.omegaRadiansPerSecond - lastOmega;
        if (Math.abs(deltaOmega) > maxDeltaOmega) {
            speeds.omegaRadiansPerSecond = lastOmega + Math.copySign(maxDeltaOmega, deltaOmega);
        }

        // --- Speed limiting (cap to AutoConstants maxes) ---
        // cap translational speed vector magnitude
        double translationalSpeed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        if (translationalSpeed > AutoConstants.kMaxSpeedMetersPerSecond) {
            double scale = AutoConstants.kMaxSpeedMetersPerSecond / translationalSpeed;
            speeds.vxMetersPerSecond *= scale;
            speeds.vyMetersPerSecond *= scale;
        }
        // cap angular speed
        if (Math.abs(speeds.omegaRadiansPerSecond) > AutoConstants.kMaxAngularSpeedRadiansPerSecond) {
            speeds.omegaRadiansPerSecond = Math.copySign(AutoConstants.kMaxAngularSpeedRadiansPerSecond, speeds.omegaRadiansPerSecond);
        }

        // store for next iteration
        lastVx = speeds.vxMetersPerSecond;
        lastVy = speeds.vyMetersPerSecond;
        lastOmega = speeds.omegaRadiansPerSecond;

        // Drive the robot
        driveFieldRelative(speeds);
    }

    // creates auto factory for determining auto trajectories -> empty sample/log and start behavior
    public AutoFactory createAutoFactory() {
        return createAutoFactory((sample, isStart) -> {
        });
    }

    public AutoFactory createAutoFactory(TrajectoryLogger<SwerveSample> trajLogger) {
        return new AutoFactory(
                this::getPose,
                this::resetOdometry,
                this::followPath,
                true, // i.e. field relative
                this,
                trajLogger
        );
    }

    public SwerveSubSystem() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                gyro.resetYaw(); // zeros heading -> NAVX3 reset() changed to resetYaw()
                frontLeft.resetTurn();
                frontRight.resetTurn();
                backLeft.resetTurn();
                backRight.resetTurn();
            } catch (Exception e) {
            }
        }).start();
    }

    /*public double getHeading() {
        return Math.IEEEremainder(-gyro.getAngle(), 360);
    }
*/
    public Rotation2d getRotation2d() {
    //    return Rotation2d.fromDegrees(getHeading());
        return gyro.getRotation2d();
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(getRotation2d(), new SwerveModulePosition[]{
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        }, pose);
        
        // reset internal velocity trackers so acceleration limiter doesn't introduce a jump
        lastVx = 0.0;
        lastVy = 0.0;
        lastOmega = 0.0;
    }

    public void driveFieldRelative(ChassisSpeeds speeds) {
        drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        speeds.vxMetersPerSecond,
                        speeds.vyMetersPerSecond,
                        speeds.omegaRadiansPerSecond,
                        getRotation2d()
                )
        );
    }

    public void drive(ChassisSpeeds speeds) {
        SwerveModuleState[] states = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);

        setModuleStates(states);
    }

    public void stop() {
        drive(new ChassisSpeeds());
    }

    @Override
    public void periodic() {
        odometer.update(getRotation2d(),
                new SwerveModulePosition[]{
                    frontLeft.getPosition(),
                    frontRight.getPosition(),
                    backLeft.getPosition(),
                    backRight.getPosition()
                });

    }

    public double getAverageDriveVelocity() {
        double aveDriveVeloAggregate = frontLeft.getDriveVelocity() + frontRight.getDriveVelocity()
                + backLeft.getDriveVelocity() + backRight.getDriveVelocity();
        return aveDriveVeloAggregate / 4;
    }

    public double getAverageTurnVelocity() {
        double aveTurnVeloAggregate = frontLeft.getTurningVelocity() + frontRight.getTurningVelocity()
                + backLeft.getTurningVelocity() + backRight.getTurningVelocity();
        return aveTurnVeloAggregate / 4;
    }

    public void disabledPeriodic() {

        SmartDashboard.putNumber("Front Left Cancoder Angle", frontLeft.getAbsoluteEncoderRad() / Math.PI * 180);
        SmartDashboard.putNumber("Back Left Cancoder Angle", backLeft.getAbsoluteEncoderRad() / Math.PI * 180);
        SmartDashboard.putNumber("Front Right Cancoder Angle", frontRight.getAbsoluteEncoderRad() / Math.PI * 180);
        SmartDashboard.putNumber("Back Right Cancoder Angle", backRight.getAbsoluteEncoderRad() / Math.PI * 180);

        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());

        SmartDashboard.putNumber("Swerve Drive Velocity", getAverageDriveVelocity());
        SmartDashboard.putNumber("Swerve Turn Velocity", getAverageTurnVelocity());

        SmartDashboard.putNumber("Front Left Drive Velocity", frontLeft.getDriveVelocity());
        SmartDashboard.putNumber("Front Right Drive Velocity", frontRight.getDriveVelocity());
        SmartDashboard.putNumber("Back Left Drive Velocity", backLeft.getDriveVelocity());
        SmartDashboard.putNumber("Back Right Drive Velocity", backRight.getDriveVelocity());

        SmartDashboard.putNumber("Front Left Turn Velocity", frontLeft.getTurningVelocity());
        SmartDashboard.putNumber("Front Right Turn Velocity", frontRight.getTurningVelocity());
        SmartDashboard.putNumber("Back Left Turn Velocity", backLeft.getTurningVelocity());
        SmartDashboard.putNumber("Back Right Turn Velocity", backRight.getTurningVelocity());
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

}
