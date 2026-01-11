package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CageClawSubSystem;

public class CageClawCmd extends Command {
    CageClawSubSystem cageClawSubSystem;
    public CageClawCmd(CageClawSubSystem cageClawSubsystem) {
        cageClawSubSystem = cageClawSubsystem;
        addRequirements(cageClawSubSystem);
    }
    // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      cageClawSubSystem.clampControl();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    cageClawSubSystem.endClawMotors(); // stop motors once interrupted
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
