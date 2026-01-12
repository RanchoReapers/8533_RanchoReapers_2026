package frc.robot;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.IntakeCmd;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.IntakeSubSystem;
import frc.robot.subsystems.LimelightDetectionSubSystem;
import frc.robot.subsystems.ShooterSubSystem;
import frc.robot.subsystems.ClimberSubSystem;
import frc.robot.subsystems.SwerveSubSystem;

public class RobotContainer {
  // Define subsystems and commands, establish auton chooser
  
  private final AutoChooser autonomousProgramChooser;

  private final AutoFactory autoFactory;

  public final static SwerveSubSystem swerveSubsystem = new SwerveSubSystem();
  public final static IntakeSubSystem intakeSubsystem = new IntakeSubSystem(14);
  // public final static ShooterSubSystem shooterSubsystem = new ShooterSubSystem(15);
  // public final static ClimberSubSystem climberSubsystem = new ClimberSubSystem(16, 17);
  // UNCOMMENT THESE WHEN ROBOT IS BUILT AND WIRED; add instantCommands
  
  public final static LimelightDetectionSubSystem limelightDetectionSubsystem = new LimelightDetectionSubSystem();

  public final static XboxController driverController = new XboxController(OIConstants.kDriverControllerPort);
  public final static XboxController operatorController = new XboxController(OIConstants.kOperatorControllerPort);
  // switch intake to go only in? operator can do the shooting with rt and elevator with lb/rb

  public final static Trigger xboxLTButtonTriggerOP = new Trigger(() -> operatorController.getRawAxis(XboxController.Axis.kLeftTrigger.value) >= 0.1); //intake in
  public final static Trigger xboxRTButtonTriggerOP = new Trigger(() -> operatorController.getRawAxis(XboxController.Axis.kRightTrigger.value) >= 0.1); //shoot out

  public final static Trigger xboxLBButtonTriggerOP = new JoystickButton(operatorController, XboxController.Button.kLeftBumper.value); //elevator down
  public final static Trigger xboxRBButtonTriggerOP = new JoystickButton(operatorController, XboxController.Button.kRightBumper.value); //elevator up

  public final static Field2d m_field = new Field2d();


  public RobotContainer() {

    autoFactory = new AutoFactory(
      swerveSubsystem::getPose, // A function that returns the current robot pose
      swerveSubsystem::resetOdometry, // A function that resets the current robot pose to the provided Pose2d
      swerveSubsystem::followTrajectory, // The drive subsystem trajectory follower 
      false, // If alliance flipping should be enabled 
      swerveSubsystem // The drive subsystem
      );
    // define program chooser
    autonomousProgramChooser = new AutoChooser(); 

    // programs:
    autonomousProgramChooser.addRoutine("TEST - Straight Line", this::straightLineAuto);
    autonomousProgramChooser.addRoutine("TEST - Curlicue", this::curlicueAuto);
    
    // add programs to Elastic
    SmartDashboard.putData("Auto Chooser", autonomousProgramChooser);

    //swerveSubsystem.setDefaultCommand(swapDriveControlMethod());

    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(swerveSubsystem,
     () -> -driverController.getRawAxis(OIConstants.kDriverYAxis), 
     () -> -driverController.getRawAxis(OIConstants.kDriverXAxis),
     () -> -driverController.getRawAxis(OIConstants.kDriverRotAxis), 
     () -> driverController.getRightBumperButton()));
    
     xboxLTButtonTriggerOP.debounce(0.1).whileTrue(callDoIntake()).whileFalse(callIntakeTriggerReleased());
     intakeSubsystem.setDefaultCommand(new IntakeCmd(intakeSubsystem));

  }

  // Define AutoRoutine (ex -- leaves starting line)
  public AutoRoutine straightLineAuto() {
    AutoRoutine routine = autoFactory.newRoutine("straightLineAuto");
    AutoTrajectory path = routine.trajectory("straightLine");

    routine.active().onTrue( //when routine starts, reset odometry -> follow trajectory
        Commands.sequence(
            path.resetOdometry(),
            path.cmd()
        )
    );
    path.done().onTrue(
        Commands.runOnce(swerveSubsystem::stop) //calls the stop function to end movement
    );
    return routine;
  }

  // Define AutoRoutine (ex -- goes in curlicue)
  public AutoRoutine curlicueAuto() {
    AutoRoutine routine = autoFactory.newRoutine("curlicueAuto");
    AutoTrajectory path = routine.trajectory("CurlicueTest");

    routine.active().onTrue( //when routine starts, reset odometry -> follow trajectory
        Commands.sequence(
            path.resetOdometry(),
            path.cmd()
        )
    );
    path.done().onTrue(
        Commands.runOnce(swerveSubsystem::stop) //calls the stop function to end movement
    );
    return routine;
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

  public Command callDoIntake() {
      return new InstantCommand(() -> intakeSubsystem.doIntake());
  }

  public Command callIntakeTriggerReleased() {
    return new InstantCommand(() -> intakeSubsystem.intakeTriggerReleased());
  }

  public Command getAutonomousCommand() {
    return autonomousProgramChooser.selectedCommand();
  }

    public void disabledPeriodic() {
        //telemetry for debugging
        swerveSubsystem.periodic();
        swerveSubsystem.disabledPeriodic();
        SmartDashboard.putBoolean("Joystick Arm State", operatorController.getAButton());
        SmartDashboard.putBoolean("Joystick Claw State", operatorController.getXButton());

        SmartDashboard.putNumber("Left Y Joystick Axis", driverController.getRawAxis(OIConstants.kDriverYAxis));
        SmartDashboard.putNumber("Left X Joystick Axis", driverController.getRawAxis(OIConstants.kDriverXAxis));
        SmartDashboard.putNumber("Right X Joystick Axis", driverController.getRawAxis(OIConstants.kDriverRotAxis));
        limelightDetectionSubsystem.periodicOdometry();
        //shooterSubsystem.shooterPeriodic();
        intakeSubsystem.intakePeriodic();
    }

    public void enabledInit() {

    }

    public void disabledInit() {
        
    }
}