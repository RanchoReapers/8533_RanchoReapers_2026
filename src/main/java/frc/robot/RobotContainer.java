package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ArmJoystickCmd;
import frc.robot.commands.CageClawCmd;
import frc.robot.commands.IntakeCmd;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.ArmSubSystem;
import frc.robot.subsystems.CageClawSubSystem;
import frc.robot.subsystems.IntakeSubSystem;
import frc.robot.subsystems.LimelightDetectionSubSystem;
import frc.robot.subsystems.SwerveSubSystem;

public class RobotContainer {
  // Define subsystems and commands
  public final static SwerveSubSystem swerveSubsystem = new SwerveSubSystem();
  public final static ArmSubSystem armSubsystem = new ArmSubSystem(14,15);
  public final static CageClawSubSystem cageClawSubsystem = new CageClawSubSystem(16);
  public final static IntakeSubSystem intakeSubsystem = new IntakeSubSystem(17);
  public final static LimelightDetectionSubSystem limelightDetectionSubsystem = new LimelightDetectionSubSystem();

  public final static XboxController driverController = new XboxController(OIConstants.kDriverControllerPort);
  public final static XboxController operatorController = new XboxController(OIConstants.kOperatorControllerPort);
  
  public final static Trigger xboxXButtonTriggerOP = new JoystickButton(operatorController, XboxController.Button.kX.value);
  public final static Trigger xboxAButtonTriggerOP = new JoystickButton(operatorController, XboxController.Button.kA.value);
  public final static Trigger xboxYButtonTriggerOP = new JoystickButton(operatorController, XboxController.Button.kY.value);

  public final static Trigger xboxLTButtonTriggerOP = new Trigger(() -> operatorController.getRawAxis(XboxController.Axis.kLeftTrigger.value) >= 0.1);
  public final static Trigger xboxRTButtonTriggerOP = new Trigger(() -> operatorController.getRawAxis(XboxController.Axis.kRightTrigger.value) >= 0.1);

  public final static Field2d m_field = new Field2d();


  public RobotContainer() {
    //swerveSubsystem.setDefaultCommand(swapDriveControlMethod());

    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(swerveSubsystem,
     () -> -driverController.getRawAxis(OIConstants.kDriverYAxis), 
     () -> -driverController.getRawAxis(OIConstants.kDriverXAxis),
     () -> -driverController.getRawAxis(OIConstants.kDriverRotAxis), 
     () -> driverController.getRightBumperButton()));
    
    
     xboxAButtonTriggerOP.debounce(0.1).onTrue(armSwitchLowVar());
     xboxYButtonTriggerOP.debounce(0.1).onTrue(callSwitchClawArmVar());
     xboxXButtonTriggerOP.debounce(0.1).onTrue(clawSwitchOpenVar());
    
     xboxLTButtonTriggerOP.debounce(0.1).whileTrue(callIntakeIn()).whileFalse(callIntakeTriggerReleased());
     xboxRTButtonTriggerOP.debounce(0.1).whileTrue(callIntakeOut()).whileFalse(callIntakeTriggerReleased());

     cageClawSubsystem.setDefaultCommand(new CageClawCmd(cageClawSubsystem));
     armSubsystem.setDefaultCommand(new ArmJoystickCmd(armSubsystem));
     intakeSubsystem.setDefaultCommand(new IntakeCmd(intakeSubsystem));
  }

  public Command swapDriveControlMethod() {
    return new ConditionalCommand(new SwerveJoystickCmd(swerveSubsystem,
    () -> limelightDetectionSubsystem.getXSpeedLimelight(), 
    () -> limelightDetectionSubsystem.getYSpeedLimelight(),
    () -> limelightDetectionSubsystem.getTurnAngleLimelight(), 
    () -> false), 
       new SwerveJoystickCmd(swerveSubsystem,
       () -> driverController.getRawAxis(OIConstants.kDriverXAxis), 
       () -> driverController.getRawAxis(OIConstants.kDriverYAxis),
       () -> driverController.getRawAxis(OIConstants.kDriverRotAxis), 
       () -> driverController.getRightBumperButton()), 
           limelightDetectionSubsystem.getAimAssistActive());
  }

  public Command callIntakeOut() {
      return new InstantCommand(() -> intakeSubsystem.intakeOut());
  }

  public Command callIntakeTriggerReleased() {
    return new InstantCommand(() -> intakeSubsystem.intakeTriggerReleased());
  }

  public Command callIntakeIn() {
      return new InstantCommand(() -> intakeSubsystem.intakeIn());
  }

  public Command armSwitchLowVar() {
      return new InstantCommand(() -> armSubsystem.switchArmLow());
  } 

  public Command clawSwitchOpenVar() {
      return new InstantCommand(() -> cageClawSubsystem.switchClawOpen());
  }

  public Command callSwitchClawArmVar() {
    return new InstantCommand(() -> armSubsystem.switchClawArmLow());
  }

  public Command getAutonomousCommand() {

    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        .setKinematics(DriveConstants.kDriveKinematics);

    // 2. Generate trajectory
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(
            new Translation2d(0.075, 0), // Slight forward movement
            new Translation2d(0. - .075, 0)), // Slight backward movement
            new Pose2d(-0.25, 0.01, Rotation2d.fromDegrees(0)), // Final backward position
        trajectoryConfig);

    // 3. Define PID controllers for tracking trajectory
    PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
    PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // 4. Construct command to follow trajectory
    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        trajectory,
        swerveSubsystem::getPose,
        DriveConstants.kDriveKinematics,
        xController,
        yController,
        thetaController,
        swerveSubsystem::setModuleStates,
        swerveSubsystem);

    // 5. Add some init and wrap-up, and return everything
    return new SequentialCommandGroup(
        new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())),
        swerveControllerCommand,
        new InstantCommand(() -> swerveSubsystem.stopModules()));
  }

    public void disabledPeriodic() {
        //telemetry for debugging
        swerveSubsystem.periodic();
        swerveSubsystem.disabledPeriodic();
        armSubsystem.periodicOdometry();
        SmartDashboard.putBoolean("Joystick Arm State", operatorController.getAButton());
        SmartDashboard.putBoolean("Joystick Claw State", operatorController.getXButton());

        cageClawSubsystem.periodicOdometry();
        SmartDashboard.putNumber("Left Y Joystick Axis", driverController.getRawAxis(OIConstants.kDriverYAxis));
        SmartDashboard.putNumber("Left X Joystick Axis", driverController.getRawAxis(OIConstants.kDriverXAxis));
        SmartDashboard.putNumber("Right X Joystick Axis", driverController.getRawAxis(OIConstants.kDriverRotAxis));
        limelightDetectionSubsystem.periodicOdometry();
    }

    public void enabledInit() {

    }

    public void disabledInit() {
        
    }
    
}