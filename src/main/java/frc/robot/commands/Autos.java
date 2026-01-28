package frc.robot.commands;

import static frc.robot.generated.ChoreoTraj.*;

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
import frc.robot.subsystems.IntakeRetractorSubSystem;

public final class Autos {

    private final SwerveSubSystem swerveSubSystem;
    private final IntakeSubSystem intakeSubsystem;
    private final ShooterSubSystem shooterSubsystem;
    private final IntakeRetractorSubSystem intakeRetractorSubsystem;

    private final AutoFactory autoFactory;
    private final AutoChooser autonomousProgramChooser;

    public Autos(SwerveSubSystem swerveSubSystem, IntakeSubSystem intakeSubsystem, ShooterSubSystem shooterSubsystem, IntakeRetractorSubSystem intakeRetractorSubsystem) {
        this.swerveSubSystem = swerveSubSystem;
        this.intakeSubsystem = intakeSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.intakeRetractorSubsystem = intakeRetractorSubsystem;

        this.autoFactory = swerveSubSystem.createAutoFactory();
        this.autonomousProgramChooser = new AutoChooser();

        autoFactory
                .bind("activateIntake", intakeSubsystem.doIntakeCmd())
                .bind("deactivateIntake", intakeSubsystem.stopIntakeCmd())
                .bind("activateShooter", shooterSubsystem.doShootCmd())
                .bind("deactivateShooter", shooterSubsystem.stopShootCmd());

    }

    public void configureAutoChooser() {
        autonomousProgramChooser.addRoutine("LEFT TRENCH -> COLLECT SHOOT READYTOCOLLECT", this::startingFrom_LEFT_TRENCH_Performing_COLLECT_SHOOT_READYTOCOLLECT_Auto);
        autonomousProgramChooser.addRoutine("LEFT BUMP -> SHOOT COLLECT SHOOT COLLECT", this::startingFrom_LEFT_BUMP_Performing_SHOOT_COLLECT_SHOOT_COLLECT_Auto);
        autonomousProgramChooser.addRoutine("LEFT BUMP -> COLLECT SHOOT COLLECT", this::startingFrom_LEFT_BUMP_Performing_COLLECT_SHOOT_COLLECT_Auto);

        autonomousProgramChooser.addRoutine("RIGHT TRENCH -> COLLECT SHOOT READYTOCOLLECT", this::startingFrom_RIGHT_TRENCH_Performing_COLLECT_SHOOT_READYTOCOLLECT_Auto);
        autonomousProgramChooser.addRoutine("RIGHT BUMP -> SHOOT COLLECT SHOOT COLLECT", this::startingFrom_RIGHT_BUMP_Performing_SHOOT_COLLECT_SHOOT_COLLECT_Auto);
        autonomousProgramChooser.addRoutine("RIGHT BUMP -> COLLECT SHOOT COLLECT", this::startingFrom_RIGHT_BUMP_Performing_COLLECT_SHOOT_COLLECT_Auto);

        SmartDashboard.putData("Auto Chooser", autonomousProgramChooser);

        RobotModeTriggers.autonomous().whileTrue(autonomousProgramChooser.selectedCommandScheduler());
    }

    // below are our competition autos NOT DONE
    private AutoRoutine startingFrom_LEFT_TRENCH_Performing_COLLECT_SHOOT_READYTOCOLLECT_Auto() {
        // declare autoRoutine name
        final AutoRoutine startingFrom_LEFT_TRENCH_Performing_COLLECT_SHOOT_READYTOCOLLECT_AutoRoutine = autoFactory.newRoutine("starting at LEFT TRENCH performing COLLECT SHOOT READYTOCOLLECT");

        // declare autoTrajectories used in this autoRoutine & sequence of commands
        final AutoTrajectory trenchToBalls = BLUELeftTrenchToLeftSideOfBalls.asAutoTraj(startingFrom_LEFT_TRENCH_Performing_COLLECT_SHOOT_READYTOCOLLECT_AutoRoutine);
        final AutoTrajectory ballsToHub = BLUELeftBallsCollectionToHubViaTrench.asAutoTraj(startingFrom_LEFT_TRENCH_Performing_COLLECT_SHOOT_READYTOCOLLECT_AutoRoutine);
        final AutoTrajectory hubToBalls = BLUELeftHubToLeftBallsCollectionViaTrench.asAutoTraj(startingFrom_LEFT_TRENCH_Performing_COLLECT_SHOOT_READYTOCOLLECT_AutoRoutine);

        // When the routine begins, reset odometry and start the first trajectory 
        startingFrom_LEFT_TRENCH_Performing_COLLECT_SHOOT_READYTOCOLLECT_AutoRoutine.active().onTrue(
                Commands.sequence(
                        intakeRetractorSubsystem.doIntakeRetractionCmd(),
                        trenchToBalls.resetOdometry(),
                        trenchToBalls.cmd()
                )
        );

        // start subsequent trajectories when the previous is done
        trenchToBalls.done().onTrue(ballsToHub.cmd());
        ballsToHub.doneDelayed(5.5).onTrue(hubToBalls.cmd()); // shooting happens during delay

        return startingFrom_LEFT_TRENCH_Performing_COLLECT_SHOOT_READYTOCOLLECT_AutoRoutine;
    }

    private AutoRoutine startingFrom_LEFT_BUMP_Performing_SHOOT_COLLECT_SHOOT_COLLECT_Auto() {
        final AutoRoutine startingFrom_LEFT_BUMP_Performing_SHOOT_COLLECT_SHOOT_COLLECT_AutoRoutine = autoFactory.newRoutine("starting at LEFT BUMP performing SHOOT COLLECT SHOOT COLLECT");

        final AutoTrajectory bumpToHub = BLUELeftBumpToHub.asAutoTraj(startingFrom_LEFT_BUMP_Performing_SHOOT_COLLECT_SHOOT_COLLECT_AutoRoutine);
        final AutoTrajectory hubToBump = BLUELeftHubToBump.asAutoTraj(startingFrom_LEFT_BUMP_Performing_SHOOT_COLLECT_SHOOT_COLLECT_AutoRoutine);
        final AutoTrajectory bumpToBalls = BLUELeftBumpToLeftSideOfBalls.asAutoTraj(startingFrom_LEFT_BUMP_Performing_SHOOT_COLLECT_SHOOT_COLLECT_AutoRoutine);
        final AutoTrajectory ballsToHub = BLUELeftBallsCollectionToHubViaBump.asAutoTraj(startingFrom_LEFT_BUMP_Performing_SHOOT_COLLECT_SHOOT_COLLECT_AutoRoutine);
        final AutoTrajectory hubToBalls = BLUELeftHubToLeftBallsCollectionViaBump.asAutoTraj(startingFrom_LEFT_BUMP_Performing_SHOOT_COLLECT_SHOOT_COLLECT_AutoRoutine);

        startingFrom_LEFT_BUMP_Performing_SHOOT_COLLECT_SHOOT_COLLECT_AutoRoutine.active().onTrue(
                Commands.sequence(
                        intakeRetractorSubsystem.doIntakeRetractionCmd(),
                        bumpToHub.resetOdometry(),
                        bumpToHub.cmd()
                )
        );

        bumpToHub.doneDelayed(2.5).onTrue(hubToBump.cmd());
        hubToBump.done().onTrue(bumpToBalls.cmd());
        bumpToBalls.done().onTrue(ballsToHub.cmd());
        ballsToHub.doneDelayed(5).onTrue(hubToBalls.cmd());

        return startingFrom_LEFT_BUMP_Performing_SHOOT_COLLECT_SHOOT_COLLECT_AutoRoutine;
    }

    private AutoRoutine startingFrom_LEFT_BUMP_Performing_COLLECT_SHOOT_COLLECT_Auto() {
        final AutoRoutine startingFrom_LEFT_BUMP_Performing_COLLECT_SHOOT_COLLECT_AutoRoutine = autoFactory.newRoutine("starting at LEFT BUMP performing SHOOT COLLECT SHOOT COLLECT");

        final AutoTrajectory bumpToBalls = BLUELeftBumpToLeftSideOfBalls.asAutoTraj(startingFrom_LEFT_BUMP_Performing_COLLECT_SHOOT_COLLECT_AutoRoutine);
        final AutoTrajectory ballsToHub = BLUELeftBallsCollectionToHubViaBump.asAutoTraj(startingFrom_LEFT_BUMP_Performing_COLLECT_SHOOT_COLLECT_AutoRoutine);
        final AutoTrajectory hubToBalls = BLUELeftHubToLeftBallsCollectionViaBump.asAutoTraj(startingFrom_LEFT_BUMP_Performing_COLLECT_SHOOT_COLLECT_AutoRoutine);

        startingFrom_LEFT_BUMP_Performing_COLLECT_SHOOT_COLLECT_AutoRoutine.active().onTrue(
                Commands.sequence(
                        intakeRetractorSubsystem.doIntakeRetractionCmd(),
                        bumpToBalls.resetOdometry(),
                        bumpToBalls.cmd()
                )
        );

        bumpToBalls.done().onTrue(ballsToHub.cmd());
        ballsToHub.doneDelayed(5).onTrue(hubToBalls.cmd());

        return startingFrom_LEFT_BUMP_Performing_COLLECT_SHOOT_COLLECT_AutoRoutine;
    }

    private AutoRoutine startingFrom_RIGHT_TRENCH_Performing_COLLECT_SHOOT_READYTOCOLLECT_Auto() {
        final AutoRoutine startingFrom_RIGHT_TRENCH_Performing_COLLECT_SHOOT_READYTOCOLLECT_AutoRoutine = autoFactory.newRoutine("starting at RIGHT TRENCH performing COLLECT SHOOT READYTOCOLLECT");

        final AutoTrajectory trenchToBalls = BLUERightTrenchToRightSideOfBalls.asAutoTraj(startingFrom_RIGHT_TRENCH_Performing_COLLECT_SHOOT_READYTOCOLLECT_AutoRoutine);
        final AutoTrajectory ballsToHub = BLUERightBallsCollectionToHubViaTrench.asAutoTraj(startingFrom_RIGHT_TRENCH_Performing_COLLECT_SHOOT_READYTOCOLLECT_AutoRoutine);
        final AutoTrajectory hubToBalls = BLUERightHubToRightBallsCollectionViaTrench.asAutoTraj(startingFrom_RIGHT_TRENCH_Performing_COLLECT_SHOOT_READYTOCOLLECT_AutoRoutine);

        startingFrom_RIGHT_TRENCH_Performing_COLLECT_SHOOT_READYTOCOLLECT_AutoRoutine.active().onTrue(
                Commands.sequence(
                        intakeRetractorSubsystem.doIntakeRetractionCmd(),
                        trenchToBalls.resetOdometry(),
                        trenchToBalls.cmd()
                )
        );

        trenchToBalls.done().onTrue(ballsToHub.cmd());
        ballsToHub.doneDelayed(5.5).onTrue(hubToBalls.cmd());

        return startingFrom_RIGHT_TRENCH_Performing_COLLECT_SHOOT_READYTOCOLLECT_AutoRoutine;
    }

    private AutoRoutine startingFrom_RIGHT_BUMP_Performing_SHOOT_COLLECT_SHOOT_COLLECT_Auto() {
        final AutoRoutine startingFrom_RIGHT_BUMP_Performing_SHOOT_COLLECT_SHOOT_COLLECT_AutoRoutine = autoFactory.newRoutine("starting at RIGHT BUMP performing SHOOT COLLECT SHOOT COLLECT");

        final AutoTrajectory bumpToHub = BLUERightBumpToHub.asAutoTraj(startingFrom_RIGHT_BUMP_Performing_SHOOT_COLLECT_SHOOT_COLLECT_AutoRoutine);
        final AutoTrajectory hubToBump = BLUERightHubToBump.asAutoTraj(startingFrom_RIGHT_BUMP_Performing_SHOOT_COLLECT_SHOOT_COLLECT_AutoRoutine);
        final AutoTrajectory bumpToBalls = BLUERightBumpToRightSideOfBalls.asAutoTraj(startingFrom_RIGHT_BUMP_Performing_SHOOT_COLLECT_SHOOT_COLLECT_AutoRoutine);
        final AutoTrajectory ballsToHub = BLUERightBallsCollectionToHubViaBump.asAutoTraj(startingFrom_RIGHT_BUMP_Performing_SHOOT_COLLECT_SHOOT_COLLECT_AutoRoutine);
        final AutoTrajectory hubToBalls = BLUERightHubToRightBallsCollectionViaBump.asAutoTraj(startingFrom_RIGHT_BUMP_Performing_SHOOT_COLLECT_SHOOT_COLLECT_AutoRoutine);

        startingFrom_RIGHT_BUMP_Performing_SHOOT_COLLECT_SHOOT_COLLECT_AutoRoutine.active().onTrue(
                Commands.sequence(
                        intakeRetractorSubsystem.doIntakeRetractionCmd(),
                        bumpToHub.resetOdometry(),
                        bumpToHub.cmd()
                )
        );

        bumpToHub.doneDelayed(2.5).onTrue(hubToBump.cmd());
        hubToBump.done().onTrue(bumpToBalls.cmd());
        bumpToBalls.done().onTrue(ballsToHub.cmd());
        ballsToHub.doneDelayed(5).onTrue(hubToBalls.cmd());

        return startingFrom_RIGHT_BUMP_Performing_SHOOT_COLLECT_SHOOT_COLLECT_AutoRoutine;
    }

    private AutoRoutine startingFrom_RIGHT_BUMP_Performing_COLLECT_SHOOT_COLLECT_Auto() {
        final AutoRoutine startingFrom_RIGHT_BUMP_Performing_COLLECT_SHOOT_COLLECT_AutoRoutine = autoFactory.newRoutine("starting at RIGHT BUMP performing SHOOT COLLECT SHOOT COLLECT");

        final AutoTrajectory bumpToBalls = BLUERightBumpToRightSideOfBalls.asAutoTraj(startingFrom_RIGHT_BUMP_Performing_COLLECT_SHOOT_COLLECT_AutoRoutine);
        final AutoTrajectory ballsToHub = BLUERightBallsCollectionToHubViaBump.asAutoTraj(startingFrom_RIGHT_BUMP_Performing_COLLECT_SHOOT_COLLECT_AutoRoutine);
        final AutoTrajectory hubToBalls = BLUERightHubToRightBallsCollectionViaBump.asAutoTraj(startingFrom_RIGHT_BUMP_Performing_COLLECT_SHOOT_COLLECT_AutoRoutine);

        startingFrom_RIGHT_BUMP_Performing_COLLECT_SHOOT_COLLECT_AutoRoutine.active().onTrue(
                Commands.sequence(
                        intakeRetractorSubsystem.doIntakeRetractionCmd(),
                        bumpToBalls.resetOdometry(),
                        bumpToBalls.cmd()
                )
        );

        bumpToBalls.done().onTrue(ballsToHub.cmd());
        ballsToHub.doneDelayed(5).onTrue(hubToBalls.cmd());

        return startingFrom_RIGHT_BUMP_Performing_COLLECT_SHOOT_COLLECT_AutoRoutine;
    }

}
