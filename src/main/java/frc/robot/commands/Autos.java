package frc.robot.commands;

import static frc.robot.generated.ChoreoTraj.CurlicueTest;
import static frc.robot.generated.ChoreoTraj.StraightLine;
import static frc.robot.generated.ChoreoTraj.StraightLineWithHeadingTurn;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.subsystems.SwerveSubSystem;

public final class Autos {
    private final SwerveSubSystem swerveSubSystem;

    private final AutoFactory autoFactory;
    private final AutoChooser autonomousProgramChooser;

    public Autos (SwerveSubSystem swerveSubSystem) {
        this.swerveSubSystem = swerveSubSystem;
    
        this.autoFactory = swerveSubSystem.createAutoFactory();
        this.autonomousProgramChooser = new AutoChooser();
    }

    public void configureAutoChooser() {
        autonomousProgramChooser.addRoutine("TEST - Straight Line", this::straightLineAuto);
        autonomousProgramChooser.addRoutine("TEST - Curlicue Test", this::curlicueAuto);
        autonomousProgramChooser.addRoutine("TEST - Straight Line With Heading Turn", this::straightLineWithHeadingTurnAuto);

        SmartDashboard.putData("Auto Chooser", autonomousProgramChooser);

        RobotModeTriggers.autonomous().whileTrue(autonomousProgramChooser.selectedCommandScheduler());
    }

    // the following 3 are examples of auto routines for testing purposes.
    private AutoRoutine straightLineAuto() {
        final AutoRoutine routine = autoFactory.newRoutine("Straight Line");
        final AutoTrajectory straightLineTrajectory = StraightLine.asAutoTraj(routine);

        routine.active().onTrue(Commands.sequence(straightLineTrajectory.resetOdometry(), straightLineTrajectory.cmd()));

        return routine;
    }

    private AutoRoutine curlicueAuto() {
        final AutoRoutine routine = autoFactory.newRoutine("Curlicue");
        final AutoTrajectory curlicueTrajectory = CurlicueTest.asAutoTraj(routine);
        
        routine.active().onTrue(Commands.sequence(curlicueTrajectory.resetOdometry(), curlicueTrajectory.cmd()));

        return routine;
    }

    private AutoRoutine straightLineWithHeadingTurnAuto() {
        final AutoRoutine routine = autoFactory.newRoutine("Straight Line with Heading Turn");
        final AutoTrajectory straightLineWithHeadingTurnTrajectory = StraightLineWithHeadingTurn.asAutoTraj(routine);
        
        routine.active().onTrue(Commands.sequence(straightLineWithHeadingTurnTrajectory.resetOdometry(), straightLineWithHeadingTurnTrajectory.cmd()));

        return routine;
    }

}
