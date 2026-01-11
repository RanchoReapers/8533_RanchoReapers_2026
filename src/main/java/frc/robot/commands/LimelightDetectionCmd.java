package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightDetectionSubSystem;
import frc.robot.subsystems.SwerveSubSystem;

public class LimelightDetectionCmd extends Command {
    LimelightDetectionSubSystem limelightDetectionSubSystem;
    SwerveSubSystem swerveSubSystem;
    public LimelightDetectionCmd(LimelightDetectionSubSystem limelightDetectionSubSystem, SwerveSubSystem swerveSubSystem) {
        this.limelightDetectionSubSystem = limelightDetectionSubSystem;
        this.swerveSubSystem = swerveSubSystem;
        addRequirements(limelightDetectionSubSystem, swerveSubSystem);
    }
    // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    limelightDetectionSubSystem.aimAssist();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}