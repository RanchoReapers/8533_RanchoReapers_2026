package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.LimelightDetectionCmd;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.LimelightDetectionSubSystem;
import frc.robot.subsystems.SwerveSubSystem;
import frc.robot.subsystems.IntakeSubSystem;
import frc.robot.subsystems.IntakeRetractorSubSystem;
import frc.robot.Constants.IntakeRetractorConstants;
import frc.robot.commands.IntakeCmd;
import frc.robot.commands.IntakeRetractorCmd;
import frc.robot.subsystems.ShooterSubSystem;
import frc.robot.commands.ShooterCmd;

public class RobotContainer {
    // Define subsystems and commands

    public final static SwerveSubSystem swerveSubsystem = new SwerveSubSystem();
    public final static IntakeSubSystem intakeSubsystem = new IntakeSubSystem(14);
    public final static IntakeRetractorSubSystem intakeRetractorSubsystem = new IntakeRetractorSubSystem(15, 16, IntakeRetractorConstants.IntakeRetractorAbsoluteEncoderOffsetRad);
    public final static ShooterSubSystem shooterSubsystem = new ShooterSubSystem(17, 18);

    public final static LimelightDetectionSubSystem limelightDetectionSubsystem = new LimelightDetectionSubSystem();

    private final Autos autos = new Autos(swerveSubsystem, intakeSubsystem, intakeRetractorSubsystem, shooterSubsystem);

    public final static XboxController driverController = new XboxController(OIConstants.kDriverControllerPort);
    public final static XboxController operatorController = new XboxController(OIConstants.kOperatorControllerPort);

    public final static Trigger xboxLTButtonTriggerOP = new Trigger(() -> operatorController.getRawAxis(XboxController.Axis.kLeftTrigger.value) >= 0.1); // intake in
    public final static Trigger xboxRTButtonTriggerOP = new Trigger(() -> operatorController.getRawAxis(XboxController.Axis.kRightTrigger.value) >= 0.1); // shoot out
    public final static Trigger xboxXButtonTriggerOP = new JoystickButton(operatorController, XboxController.Button.kX.value); // intake retractor toggle

    public final static Trigger xboxXButtonTriggerDriver = new JoystickButton(driverController, XboxController.Button.kX.value); // aim assist toggle
    public final static Trigger xboxYButtonTriggerDriver = new JoystickButton(driverController, XboxController.Button.kY.value); // limelight pipeline toggle
    public final static Trigger xboxRTButtonTriggerDriver = new Trigger(() -> driverController.getRawAxis(XboxController.Axis.kRightTrigger.value) >= 0.1); // intake unstuck (out) - overrides operator intake commands

    public RobotContainer() {

        autos.configureAutoChooser();

        swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                swerveSubsystem,
                () -> driverController.getRawAxis(OIConstants.kDriverYAxis),
                () -> driverController.getRawAxis(OIConstants.kDriverXAxis),
                () -> driverController.getRawAxis(OIConstants.kDriverRotAxis),
                () -> operatorController.getRawAxis(OIConstants.kDriverYAxis), // axes remain the same
                () -> operatorController.getRawAxis(OIConstants.kDriverXAxis),
                () -> operatorController.getRawAxis(OIConstants.kDriverRotAxis),
                () -> driverController.getRightBumperButton(),
                () -> !limelightDetectionSubsystem.limelightOverrideActive, // returns TRUE when OVERRIDE is ACTIVE (Limelight DISABLED) -> pass as FALSE as SwerveJoystickCmd asks if aim assist is ENABLED or not
                limelightDetectionSubsystem));

        xboxLTButtonTriggerOP.debounce(0.1).whileTrue(callDoIntake()).whileFalse(callIntakeTriggerReleased());
        intakeSubsystem.setDefaultCommand(new IntakeCmd(intakeSubsystem));

        xboxXButtonTriggerOP.debounce(0.1).onTrue(callDoIntakeRetraction());
        intakeRetractorSubsystem.setDefaultCommand(new IntakeRetractorCmd(intakeRetractorSubsystem));
        
        xboxXButtonTriggerDriver.debounce(0.1).onTrue(callSwapLimelightOverride());
        xboxYButtonTriggerDriver.debounce(0.1).onTrue(callToggleLimelightPipeline());
        limelightDetectionSubsystem.setDefaultCommand(new LimelightDetectionCmd(limelightDetectionSubsystem));

        xboxRTButtonTriggerOP.debounce(0.1).whileTrue(callDoShoot()).whileFalse(callShooterTriggerReleased());
        shooterSubsystem.setDefaultCommand(new ShooterCmd(shooterSubsystem));

        xboxRTButtonTriggerDriver.debounce(0.1).whileTrue(callDoIntakeUnstuck()).onFalse(callIntakeUnstuckTriggerReleased());
    }

    public final Command callDoIntake() {
        return new InstantCommand(() -> intakeSubsystem.doIntake());
    }

    public final Command callIntakeTriggerReleased() {
        return new InstantCommand(() -> intakeSubsystem.intakeTriggerReleased());
    }
    
    public final Command callDoShoot() {
        return new InstantCommand(() -> shooterSubsystem.doShoot());
    }

    public final Command callShooterTriggerReleased() {
        return new InstantCommand(() -> shooterSubsystem.shooterTriggerReleased());
    }

    public final Command callDoIntakeUnstuck() {
        return new InstantCommand(() -> intakeSubsystem.doIntakeUnsticking());
    }

    public final Command callIntakeUnstuckTriggerReleased() {
        return new InstantCommand(() -> intakeSubsystem.intakeUnstickingTriggerReleased());
    }
    
    public final Command callDoIntakeRetraction() {
        return new InstantCommand(() -> intakeRetractorSubsystem.doIntakeRetraction());
    }

    public final Command callSwapLimelightOverride() {
        return new InstantCommand(() -> limelightDetectionSubsystem.swapLimelightOverrideActive());
    }

    public final Command callToggleLimelightPipeline() {
        return new InstantCommand(() -> limelightDetectionSubsystem.swapPipelineIndex());
    }

    public void disabledPeriodic() {
        swerveSubsystem.periodic();
        swerveSubsystem.disabledPeriodic();
        limelightDetectionSubsystem.periodicOdometry();
        shooterSubsystem.shooterPeriodic();
        intakeSubsystem.intakePeriodic();
        intakeRetractorSubsystem.intakeRetractorPeriodic();
    }

    public void enabledInit() {

    }

    public void disabledInit() {

    }
}
