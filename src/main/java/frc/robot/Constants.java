package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {

    public final class ArmConstants {
        public static final double ArmVoltage = 6;
    }

    public final class CageClawConstants {
        public static final double CageClawVoltage = 2.5;
    }

    public final class IntakeConstants {
        public static final double IntakeVoltage = 3;
    }

    public final class ModuleConstants {
        public static final double kDriveEncoderRot2Meter = 0.148148148 * Math.PI * 0.1; // Drive motor gear ratio * PI * Robot wheel diameter (in meters)
        public static final double kTurnEncoderRot2Rad = 0.04666666 * Math.PI * 2; // Turn motor gear ratio converted to radians
    }

    public static final class DriveConstants {
        // SECTION - Base measurements
        // NOTE - CHANGE kTrackWidth & kWheelBase IF YOU CHANGE BASE MEASUREMENTS
        public static final double kTrackWidth = Units.inchesToMeters(23);
        // Distance between right and left wheels ^

        public static final double kWheelBase = Units.inchesToMeters(21.2);
        // Distance between front and back wheels ^

        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));
        // !SECTION

        // SECTION - SparkMAX CAN ID's for swerve
        public static final int kFrontLeftDriveMotorPort = 4;
        public static final int kFrontLeftTurningMotorPort = 5;
        public static final int kFrontRightDriveMotorPort = 7;
        public static final int kFrontRightTurningMotorPort = 6;

        public static final int kBackLeftDriveMotorPort = 3;
        public static final int kBackLeftTurningMotorPort = 2;
        public static final int kBackRightDriveMotorPort = 9;
        public static final int kBackRightTurningMotorPort = 8;
        // !SECTION

        // SECTION - Encoder ports for CANcoders (swerve drive motors)
        public static final int kBackRightDriveAbsoluteEncoderPort = 13;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 10;
        public static final int kFrontLeftDriveAbsoluteEncoderPort = 11;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 12;
        // !SECTION

        // SECTION - Defines whether or not to invert drive & turn motors
        public static final boolean kFrontLeftDriveEncoderReversed = false;
        public static final boolean kFrontRightDriveEncoderReversed = false;
        public static final boolean kBackLeftDriveEncoderReversed = false;
        public static final boolean kBackRightDriveEncoderReversed = false;

        public static final boolean kFrontLeftTurningEncoderReversed = true;
        public static final boolean kFrontRightTurningEncoderReversed = true;
        public static final boolean kBackLeftTurningEncoderReversed = true;
        public static final boolean kBackRightTurningEncoderReversed = true;
        // !SECTION

        // SECTION - Offset values for swerve CANCoders
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 0.409668 - (-0.45725); // move stick forward to get first number; subtract # when manually aligned
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = -0.127686 - (-0.494500);
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = -0.375 - 0.165039;
        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = -0.33000 - 0.236572;
        // !SECTION

        // SECTION - Chassis offset reference
        public static final double kPhysicalMaxSpeedMetersPerSecond = 4.4;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 1.5;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;

        public static final double kSlowButtonDriveModifier = 0.25;
        public static final double kSlowButtonTurnModifier = 0.5;
        // !SECTION
    }

    public static final class AutoConstants {
        // SECTION - Auto constants (not currently being used -- may be implemented later)
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 8;
        public static final double kMaxAngularSpeedRadiansPerSecond = DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
        public static final double kPXController = 1.5;
        public static final double kPYController = 1.5;
        public static final double kPThetaController = 3;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, 
            kMaxAngularAccelerationRadiansPerSecondSquared);
            // !SECTION
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;

        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 4;

        public static final double kDeadband = 0.16;
    }

}
