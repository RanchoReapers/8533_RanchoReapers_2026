package frc.robot.subsystems;

import com.studica.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

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

    private final AHRS gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);

    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(
        DriveConstants.kDriveKinematics, new Rotation2d(),
        new SwerveModulePosition[] {
          frontLeft.getPosition(),
          frontRight.getPosition(),
          backLeft.getPosition(),
          backRight.getPosition()
        }, new Pose2d(5.0, 13.5, new Rotation2d()));

    public SwerveSubSystem()  {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                gyro.reset(); // zeros heading
                frontLeft.resetTurn();
                frontRight.resetTurn();
                backLeft.resetTurn();
                backRight.resetTurn();
            }catch (Exception e) {
            }
        }).start                                                                                                                                                                                                                                            ();
    }

    public double getHeading(){
        return Math.IEEEremainder(-gyro.getAngle(), 360);
    }

    public Rotation2d getRotation2d(){
        return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(getRotation2d(), new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        }, pose);
    }

    @Override
    public void periodic() {
        
        odometer.update(getRotation2d(),
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        });
    }

    public double getAverageDriveVelocity() {
        double aveDriveVeloAggregate = frontLeft.getDriveVelocity() + frontRight.getDriveVelocity() + backLeft.getDriveVelocity() + backRight.getDriveVelocity();
        return aveDriveVeloAggregate / 4;
    }

    public double getAverageTurnVelocity() {
        double aveTurnVeloAggregate = frontLeft.getTurningVelocity() + frontRight.getTurningVelocity() + backLeft.getTurningVelocity() + backRight.getTurningVelocity();
        return aveTurnVeloAggregate / 4;
    }

    public void disabledPeriodic() {
        SmartDashboard.putNumber("Heading", getHeading());

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

    public void setModuleStates(SwerveModuleState[] desiredStates){
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }


}