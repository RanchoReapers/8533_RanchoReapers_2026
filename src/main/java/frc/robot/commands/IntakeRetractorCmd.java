package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeRetractorSubSystem;

public class IntakeRetractorCmd extends Command {

    IntakeRetractorSubSystem intakeRetractorSubSystem;
    public IntakeRetractorCmd(IntakeRetractorSubSystem intakeRetractorSubSystem) {
        this.intakeRetractorSubSystem = intakeRetractorSubSystem;
        addRequirements(intakeRetractorSubSystem);
    }
    // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeRetractorSubSystem.intakeRetractorControl();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeRetractorSubSystem.endIntakeRetractionMotor(); // stop motors once interrupted
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
