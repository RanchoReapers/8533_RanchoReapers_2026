package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubSystem;

public class ShooterCmd extends Command {

    ShooterSubSystem shooterSubSystem;
    public ShooterCmd(ShooterSubSystem shooterSubSystem) {
        this.shooterSubSystem = shooterSubSystem;
        addRequirements(shooterSubSystem);
    }
    // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterSubSystem.shooterControl();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubSystem.endShooterMotor(); // stop motors once interrupted
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
