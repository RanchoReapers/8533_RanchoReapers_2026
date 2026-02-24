package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.LimelightDetectionSubSystem;
import frc.robot.subsystems.SwerveSubSystem;

public class RobotContainer {
    // Define subsystems and commands

    public final static SwerveSubSystem swerveSubsystem = new SwerveSubSystem();
    //public final static IntakeSubSystem intakeSubsystem = new IntakeSubSystem(14);
    //public final static IntakeRetractorSubSystem intakeRetractorSubsystem = new IntakeRetractorSubSystem(15, 16, IntakeRetractorConstants.IntakeRetractorAbsoluteEncoderOffsetRad);
    //public final static ShooterSubSystem shooterSubsystem = new ShooterSubSystem(17, 18);

    public final static LimelightDetectionSubSystem limelightDetectionSubsystem = new LimelightDetectionSubSystem();

    private final Autos autos = new Autos(swerveSubsystem/*, intakeSubsystem, shooterSubsystem, intakeRetractorSubsystem*/);

    public final static XboxController driverController = new XboxController(OIConstants.kDriverControllerPort);
    public final static XboxController operatorController = new XboxController(OIConstants.kOperatorControllerPort);

    public final static Trigger xboxLTButtonTriggerOP = new Trigger(() -> operatorController.getRawAxis(XboxController.Axis.kLeftTrigger.value) >= 0.1); // intake in
    public final static Trigger xboxRTButtonTriggerOP = new Trigger(() -> operatorController.getRawAxis(XboxController.Axis.kRightTrigger.value) >= 0.1); // shoot out

    public final static Trigger xboxLBButtonTriggerOP = new JoystickButton(operatorController, XboxController.Button.kLeftBumper.value); // climber down
    public final static Trigger xboxRBButtonTriggerOP = new JoystickButton(operatorController, XboxController.Button.kRightBumper.value); // climber up
    public final static Trigger xboxXButtonTriggerOP = new JoystickButton(operatorController, XboxController.Button.kX.value); // intake retractor toggle

    public RobotContainer() {

        autos.configureAutoChooser();

        //swerveSubsystem.setDefaultCommand(swapDriveControlMethod());
        swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(swerveSubsystem,
                () -> driverController.getRawAxis(OIConstants.kDriverYAxis),
                () -> driverController.getRawAxis(OIConstants.kDriverXAxis),
                () -> driverController.getRawAxis(OIConstants.kDriverRotAxis),
                () -> driverController.getRightBumperButton()));

        /* xboxLTButtonTriggerOP.debounce(0.1).whileTrue(callDoIntake()).whileFalse(callIntakeTriggerReleased());
        intakeSubsystem.setDefaultCommand(new IntakeCmd(intakeSubsystem));

        xboxRTButtonTriggerOP.debounce(0.1).whileTrue(callDoShoot()).whileFalse(callShooterTriggerReleased());
        shooterSubsystem.setDefaultCommand(new ShooterCmd(shooterSubsystem));

        xboxXButtonTriggerOP.debounce(0.1).onTrue(callDoIntakeRetraction());
        intakeRetractorSubsystem.setDefaultCommand(new IntakeRetractorCmd(intakeRetractorSubsystem));*/

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

    /*public Command callDoIntake() {
        return new InstantCommand(() -> intakeSubsystem.doIntake());
    }

    public Command callIntakeTriggerReleased() {
        return new InstantCommand(() -> intakeSubsystem.intakeTriggerReleased());
    }

    public Command callDoShoot() {
        return new InstantCommand(() -> shooterSubsystem.doShoot());
    }

    public Command callShooterTriggerReleased() {
        return new InstantCommand(() -> shooterSubsystem.shooterTriggerReleased());
    }
    
    public Command callDoIntakeRetraction() {
        return new InstantCommand(() -> intakeRetractorSubsystem.doIntakeRetraction());
    } */


    public void generalPeriodic() {
        swerveSubsystem.periodic();
        swerveSubsystem.disabledPeriodic();
        limelightDetectionSubsystem.periodicOdometry();
        //shooterSubsystem.shooterPeriodic();
        //intakeSubsystem.intakePeriodic();
        //intakeRetractorSubsystem.intakeRetractorPeriodic();
        // UNCOMMENT THESE WHEN ROBOT IS BUILT AND WIRED
    }

    public void enabledInit() {

    }

    public void disabledInit() {

    }
}
