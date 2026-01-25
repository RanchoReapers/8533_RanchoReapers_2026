package frc.robot.commands;

import static frc.robot.generated.ChoreoTraj.BLUELeftBallsCollectionToHubViaTrench;
import static frc.robot.generated.ChoreoTraj.BLUELeftHubToLeftBallsCollectionViaTrench;
import static frc.robot.generated.ChoreoTraj.BLUELeftTrenchToLeftSideOfBalls;
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
import frc.robot.subsystems.IntakeSubSystem;
import frc.robot.subsystems.ShooterSubSystem;
import frc.robot.subsystems.SwerveSubSystem;

public final class Autos {

    private final SwerveSubSystem swerveSubSystem;
    //private final IntakeSubSystem intakeSubsystem;
    //private final ShooterSubSystem shooterSubsystem;

    private final AutoFactory autoFactory;
    private final AutoChooser autonomousProgramChooser;

    public Autos(SwerveSubSystem swerveSubSystem/*, IntakeSubSystem intakeSubsystem, ShooterSubSystem shooterSubsystem*/) {
        this.swerveSubSystem = swerveSubSystem;
        //this.intakeSubsystem = intakeSubsystem;
        //this.shooterSubsystem = shooterSubsystem;

        this.autoFactory = swerveSubSystem.createAutoFactory();
        this.autonomousProgramChooser = new AutoChooser();

        /*autoFactory
                .bind("activateIntake", intakeSubsystem.doIntakeCmd())
                .bind("deactivateIntake", intakeSubsystem.stopIntakeCmd())
                .bind("activateShooter", shooterSubsystem.doShootCmd())
                .bind("deactivateShooter", shooterSubsystem.stopShootCmd());
                */
        
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
        final AutoRoutine straightLineAutoRoutine = autoFactory.newRoutine("Straight Line");
        final AutoTrajectory straightLineTrajectory = StraightLine.asAutoTraj(straightLineAutoRoutine);

        // When the routine begins, reset odometry and start the first trajectory 
        straightLineAutoRoutine.active().onTrue(
                Commands.sequence(
                        straightLineTrajectory.resetOdometry(),
                        straightLineTrajectory.cmd()
                )
        );

        return straightLineAutoRoutine;
    }

    private AutoRoutine curlicueAuto() {
        final AutoRoutine curlicueAutoRoutine = autoFactory.newRoutine("Curlicue");
        final AutoTrajectory curlicueTrajectory = CurlicueTest.asAutoTraj(curlicueAutoRoutine);

        // When the routine begins, reset odometry and start the first trajectory 
        curlicueAutoRoutine.active().onTrue(
                Commands.sequence(
                        curlicueTrajectory.resetOdometry(),
                        curlicueTrajectory.cmd()
                )
        );

        return curlicueAutoRoutine;
    }

    private AutoRoutine straightLineWithHeadingTurnAuto() {
        final AutoRoutine straightLineWithHeadingTurnAutoRoutine = autoFactory.newRoutine("Straight Line with Heading Turn");
        final AutoTrajectory straightLineWithHeadingTurnTrajectory = StraightLineWithHeadingTurn.asAutoTraj(straightLineWithHeadingTurnAutoRoutine);

        // When the routine begins, reset odometry and start the first trajectory 
        straightLineWithHeadingTurnAutoRoutine.active().onTrue(
                Commands.sequence(
                        straightLineWithHeadingTurnTrajectory.resetOdometry(),
                        straightLineWithHeadingTurnTrajectory.cmd()
                )
        );

        return straightLineWithHeadingTurnAutoRoutine;
    }

    // below are our competition autos NOT DONE
    private AutoRoutine startingFrom_LEFT_TRENCH_Performing_COLLECT_SHOOT_READYTOCOLLECT_Auto() {
        // declare autoRoutine Name
        final AutoRoutine startingFrom_LEFT_TRENCH_Performing_COLLECT_SHOOT_READYTOCOLLECT_AutoRoutine = autoFactory.newRoutine("starting at LEFT TRENCH performing COLLECT SHOOT READYTOCOLLECT");

        // declare autoTrajectories used in this autoRoutine & sequence of commands
        final AutoTrajectory trenchToBalls = BLUELeftTrenchToLeftSideOfBalls.asAutoTraj(startingFrom_LEFT_TRENCH_Performing_COLLECT_SHOOT_READYTOCOLLECT_AutoRoutine);
        final AutoTrajectory ballsToHubViaTrench = BLUELeftBallsCollectionToHubViaTrench.asAutoTraj(startingFrom_LEFT_TRENCH_Performing_COLLECT_SHOOT_READYTOCOLLECT_AutoRoutine);
        final AutoTrajectory hubToBallsViaTrench = BLUELeftHubToLeftBallsCollectionViaTrench.asAutoTraj(startingFrom_LEFT_TRENCH_Performing_COLLECT_SHOOT_READYTOCOLLECT_AutoRoutine);

        // When the routine begins, reset odometry and start the first trajectory 
        startingFrom_LEFT_TRENCH_Performing_COLLECT_SHOOT_READYTOCOLLECT_AutoRoutine.active().onTrue(
                Commands.sequence(
                        trenchToBalls.resetOdometry(),
                        trenchToBalls.cmd()
                )
        );

        // start subsequent trajectories when the previous is done
        trenchToBalls.done().onTrue(ballsToHubViaTrench.cmd());
        ballsToHubViaTrench.doneDelayed(5.5).onTrue(hubToBallsViaTrench.cmd()); // shooting happens during delay

        return startingFrom_LEFT_TRENCH_Performing_COLLECT_SHOOT_READYTOCOLLECT_AutoRoutine;
    }

}
