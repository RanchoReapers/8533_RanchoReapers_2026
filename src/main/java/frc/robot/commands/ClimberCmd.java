package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubSystem;

public class ClimberCmd extends Command {

    ClimberSubSystem climberSubSystem;
    public ClimberCmd(ClimberSubSystem climberSubSystem) {
        this.climberSubSystem = climberSubSystem;
        addRequirements(climberSubSystem);
    }
    // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climberSubSystem.elevatorControl();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climberSubSystem.endElevatorMotors(); // stop motors once interrupted
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
