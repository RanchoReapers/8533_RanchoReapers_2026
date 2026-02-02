package frc.robot;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.SwerveSubSystem;
import frc.robot.subsystems.LimelightDetectionSubSystem;
import frc.robot.commands.IntakeCmd;
import frc.robot.commands.ShooterCmd;
import frc.robot.commands.IntakeRetractorCmd;
import frc.robot.commands.ClimberCmd;
import frc.robot.subsystems.IntakeSubSystem;
import frc.robot.subsystems.ShooterSubSystem;
import frc.robot.subsystems.IntakeRetractorSubSystem;
import frc.robot.subsystems.ClimberSubSystem;
import frc.robot.Constants.IntakeRetractorConstants;

import frc.robot.commands.Autos;

public class RobotContainer {
    // Define subsystems and commands

    public final static SwerveSubSystem swerveSubsystem = new SwerveSubSystem();
    /*public final static IntakeSubSystem intakeSubsystem = new IntakeSubSystem(14);
    public final static ShooterSubSystem shooterSubsystem = new ShooterSubSystem(15);
    public final static IntakeRetractorSubSystem intakeRetractorSubsystem = new IntakeRetractorSubSystem(16, 17, IntakeRetractorConstants.IntakeRetractorAbsoluteEncoderOffsetRad);
    *///public final static ClimberSubSystem climberSubsystem = new ClimberSubSystem(21, 22);

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

        /*xboxLTButtonTriggerOP.debounce(0.1).whileTrue(callDoIntake()).whileFalse(callIntakeTriggerReleased());
        intakeSubsystem.setDefaultCommand(new IntakeCmd(intakeSubsystem));

        xboxRTButtonTriggerOP.debounce(0.1).whileTrue(callDoShoot()).whileFalse(callShooterTriggerReleased());
        shooterSubsystem.setDefaultCommand(new ShooterCmd(shooterSubsystem));

        xboxXButtonTriggerOP.debounce(0.1).onTrue(callDoIntakeRetraction());
        intakeRetractorSubsystem.setDefaultCommand(new IntakeRetractorCmd(intakeRetractorSubsystem));*/

        // add commands for climber
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
    }*/

    // add commands for climber
    public void disabledPeriodic() {
        swerveSubsystem.periodic();
        swerveSubsystem.disabledPeriodic();
        limelightDetectionSubsystem.periodicOdometry();
        //shooterSubsystem.shooterPeriodic();
        //climberSubsystem.climberPeriodic();
        //intakeSubsystem.intakePeriodic();
        //intakeRetractorSubsystem.intakeRetractorPeriodic();
        // UNCOMMENT THESE WHEN ROBOT IS BUILT AND WIRED
    }

    public void enabledInit() {

    }

    public void disabledInit() {

    }
}
