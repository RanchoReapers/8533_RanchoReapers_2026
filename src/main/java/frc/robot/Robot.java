// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import static frc.robot.RobotContainer.intakeSubsystem;
// import static frc.robot.RobotContainer.shooterSubsystem;
// UNCOMMENT THESE WHEN ROBOT IS BUILT AND WIRED
import static frc.robot.RobotContainer.swerveSubsystem;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import java.util.Optional;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.util.Color;

public class Robot extends TimedRobot {

    private final Alert hubStatus = new Alert("Please connect to FMS to use Alliance Switching Alerts", AlertType.kWarning);
    private final Alert corruptFMSData = new Alert("FMS Data is corrupt; alliance switching alerts CANNOT BE TRUSTED.", AlertType.kError);
    private RobotContainer m_robotContainer;
    public final Field2d m_field = new Field2d();
    public String gameData;

    // Add enum/state to replace boolean myHubActive
    private enum HubState { ACTIVE, INACTIVE, FIVE_SECOND_WARNING }
    private HubState hubState = HubState.INACTIVE;

    @Override
    public void robotInit() {
        m_robotContainer = new RobotContainer();

        SmartDashboard.putData("Field", m_field);

        SmartDashboard.putData(CommandScheduler.getInstance());
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        SmartDashboard.putNumber("CAN Utilization %", RobotController.getCANStatus().percentBusUtilization * 100.0);
        SmartDashboard.putNumber("Voltage", RobotController.getBatteryVoltage());
        SmartDashboard.putNumber("CPU Temp", RobotController.getCPUTemp());
        SmartDashboard.putBoolean("RSL", RobotController.getRSLState());
        SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
        m_robotContainer.disabledPeriodic();

        m_field.setRobotPose(RobotContainer.swerveSubsystem.getPose());

        double matchTime = DriverStation.getMatchTime();
        double intensity;

        // Rumble logic: ramp up from 0 to 1 between 32s and 30s, hold at 1 between 30s and 29s, then off -> Endgame warning
        if (matchTime >= 30.0 && matchTime <= 32.0) {
            intensity = 1.0 - ((matchTime - 30.0) / 2.0);

        } else if (matchTime < 30.0 && matchTime >= 29.0) {
            intensity = 1.0;

        } else {
            intensity = 0.0;
        }

        intensity = Math.min(Math.max(intensity, 0.0), 1.0);

        RobotContainer.driverController.setRumble(RumbleType.kBothRumble, intensity);
        RobotContainer.operatorController.setRumble(RumbleType.kBothRumble, intensity);
    }

    @Override
    public void autonomousPeriodic() {
        swerveSubsystem.periodic();
    }

    @Override
    public void teleopInit() {
        intakeSubsystem.intakeMotorStopped = true;
        // shooterSubsystem.shooterMotorStopped = true;
        // UNCOMMENT THESE WHEN ROBOT IS BUILT AND WIRED

        hubStatus.set(false);
        corruptFMSData.set(false);
    }

    @Override
    public void teleopPeriodic() {
        gameData = DriverStation.getGameSpecificMessage();
        int matchTimeTruncated = (int) Math.ceil(DriverStation.getMatchTime());
        Optional<Alliance> ally = DriverStation.getAlliance();

        switch (matchTimeTruncated) {
            case 140 -> {
                // TRANSITION SHIFT
                hubStatus.setText("Both HUBS will be active for the next 10 seconds.");
                hubState = HubState.ACTIVE;
                corruptFMSData.set(false);
            }
            case 135 -> {
                // 5 SECOND WARNING BEFORE SHIFT 1
                if (gameData.length() > 0 && ally.isPresent()) {
                    if (ally.get() == Alliance.Red) {
                        switch (gameData.charAt(0)) {
                            case 'B' -> {
                                hubStatus.setText("Your HUB will remain ACTIVE in 5 seconds.");
                                corruptFMSData.set(false);
                            }
                            case 'R' -> {
                                hubStatus.setText("Your HUB will become INACTIVE in 5 seconds.");
                                hubState = HubState.FIVE_SECOND_WARNING;
                                corruptFMSData.set(false);
                            }
                            default ->
                                corruptFMSData.set(true);
                        }
                    } else if (ally.get() == Alliance.Blue) {
                        switch (gameData.charAt(0)) {
                            case 'B' -> {
                                hubStatus.setText("Your HUB will become INACTIVE in 5 seconds.");
                                hubState = HubState.FIVE_SECOND_WARNING;
                                corruptFMSData.set(false);
                            }
                            case 'R' -> {
                                hubStatus.setText("Your HUB will remain ACTIVE in 5 seconds.");
                                corruptFMSData.set(false);
                            }
                            default ->
                                corruptFMSData.set(true);
                        }
                    }
                }
            }
            case 130 -> {
                // SHIFT 1
                if (gameData.length() > 0 && ally.isPresent()) {
                    if (ally.get() == Alliance.Red) {
                        switch (gameData.charAt(0)) {
                            case 'B' -> {
                                hubStatus.setText("Your HUB is now ACTIVE. BLUE ALLIANCE scored more FUEL during the AUTONOMOUS period.");
                                hubState = HubState.ACTIVE;
                                corruptFMSData.set(false);
                            }
                            case 'R' -> {
                                hubStatus.setText("Your HUB is now INACTIVE. RED ALLIANCE scored more FUEL during the AUTONOMOUS period.");
                                hubState = HubState.INACTIVE;
                                corruptFMSData.set(false);
                            }
                            default -> corruptFMSData.set(true);
                        }
                    } else if (ally.get() == Alliance.Blue) {
                        switch (gameData.charAt(0)) {
                            case 'B' -> {
                                hubStatus.setText("Your HUB is now INACTIVE. BLUE ALLIANCE scored more FUEL during the AUTONOMOUS period.");
                                hubState = HubState.INACTIVE;
                                corruptFMSData.set(false);
                            }
                            case 'R' -> {
                                hubStatus.setText("Your HUB is now ACTIVE. RED ALLIANCE scored more FUEL during the AUTONOMOUS period.");
                                hubState = HubState.ACTIVE;
                                corruptFMSData.set(false);
                            }
                            default -> corruptFMSData.set(true);
                        }
                    }
                }
            }
            case 110, 60 -> {
                // 5 SECOND WARNING BEFORE SHIFT 2 & 4
                hubState = HubState.FIVE_SECOND_WARNING;
                if (gameData.length() > 0 && ally.isPresent()) {
                    if (ally.get() == Alliance.Red) {
                        switch (gameData.charAt(0)) {
                            case 'B' -> {
                                hubStatus.setText("Your HUB will become INACTIVE in 5 seconds.");
                                corruptFMSData.set(false);
                            }
                            case 'R' -> {
                                hubStatus.setText("Your HUB will become ACTIVE in 5 seconds.");
                                corruptFMSData.set(false);
                           }
                            default ->
                                corruptFMSData.set(true);
                        }
                    } else if (ally.get() == Alliance.Blue) {
                        switch (gameData.charAt(0)) {
                            case 'B' -> {
                                hubStatus.setText("Your HUB will become ACTIVE in 5 seconds.");
                                corruptFMSData.set(false);
                            }
                            case 'R' -> {
                                hubStatus.setText("Your HUB will become INACTIVE in 5 seconds.");
                                corruptFMSData.set(false);
                            }
                            default ->
                                corruptFMSData.set(true);
                        }
                    }
                }
            }
            case 105, 55 -> {
                // SHIFT 2 & 4
                if (gameData.length() > 0 && ally.isPresent()) {
                    if (ally.get() == Alliance.Red) {
                        switch (gameData.charAt(0)) {
                            case 'B' -> {
                                hubStatus.setText("Your HUB is now INACTIVE.");
                                hubState = HubState.INACTIVE;
                                corruptFMSData.set(false);
                            }
                            case 'R' -> {
                                hubStatus.setText("Your HUB is now ACTIVE.");
                                hubState = HubState.ACTIVE;
                                corruptFMSData.set(false);
                            }
                            default -> corruptFMSData.set(true);
                        }
                    } else if (ally.get() == Alliance.Blue) {
                        switch (gameData.charAt(0)) {
                            case 'B' -> {
                                hubStatus.setText("Your HUB is now ACTIVE");
                                hubState = HubState.ACTIVE;
                                corruptFMSData.set(false);
                            }
                            case 'R' -> {
                                hubStatus.setText("Your HUB is now INACTIVE");
                                hubState = HubState.INACTIVE;
                                corruptFMSData.set(false);
                            }
                            default -> corruptFMSData.set(true);
                        }
                    }
                }
            }
            case 85 -> {
                // 5 SECOND WARNING BEFORE SHIFT 3
                hubState = HubState.FIVE_SECOND_WARNING;
                if (gameData.length() > 0 && ally.isPresent()) {
                    if (ally.get() == Alliance.Red) {
                        switch (gameData.charAt(0)) {
                            case 'B' -> {
                                hubStatus.setText("Your HUB will become ACTIVE in 5 seconds.");
                                corruptFMSData.set(false);
                            }
                            case 'R' -> {
                                hubStatus.setText("Your HUB will become INACTIVE in 5 seconds.");
                                corruptFMSData.set(false);
                            }
                            default ->
                                corruptFMSData.set(true);
                        }
                    } else if (ally.get() == Alliance.Blue) {
                        switch (gameData.charAt(0)) {
                            case 'B' -> {
                                hubStatus.setText("Your HUB will become INACTIVE in 5 seconds.");
                                corruptFMSData.set(false);
                            }
                            case 'R' -> {
                                hubStatus.setText("Your HUB will become ACTIVE in 5 seconds.");
                                corruptFMSData.set(false);
                            }
                            default ->
                                corruptFMSData.set(true);
                        }
                    }
                }
            }
            case 80 -> {
                // SHIFT 3
                if (gameData.length() > 0 && ally.isPresent()) {
                    if (ally.get() == Alliance.Red) {
                        switch (gameData.charAt(0)) {
                            case 'B' -> {
                                hubStatus.setText("Your HUB is now ACTIVE.");
                                hubState = HubState.ACTIVE;
                                corruptFMSData.set(false);
                            }
                            case 'R' -> {
                                hubStatus.setText("Your HUB is now INACTIVE.");
                                hubState = HubState.INACTIVE;
                                corruptFMSData.set(false);
                            }
                            default -> corruptFMSData.set(true);
                        }
                    } else if (ally.get() == Alliance.Blue) {
                        switch (gameData.charAt(0)) {
                            case 'B' -> {
                                hubStatus.setText("Your HUB is now INACTIVE.");
                                hubState = HubState.INACTIVE;
                                corruptFMSData.set(false);
                            }
                            case 'R' -> {
                                hubStatus.setText("Your HUB is now ACTIVE.");
                                hubState = HubState.ACTIVE;
                                corruptFMSData.set(false);
                            }
                            default -> corruptFMSData.set(true);
                        }
                    }
                }
            }
            case 30 -> {
                // ENDGAME
                hubStatus.setText("Both HUBS will be active for the remainder of the match (30s).");
                hubState = HubState.ACTIVE;
                corruptFMSData.set(false);
            }
            default -> {
            }
        }

        hubStatus.set(true);

        // Publish hub color: green when ACTIVE, grey when INACTIVE, flashing yellow during FIVE_SECOND_WARNING
        Color green = new Color(0, 255, 0);
        Color grey = new Color(128, 128, 128);
        Color yellow = new Color(255, 255, 0);

        Color hubColor;

        switch(hubState) {
            case ACTIVE -> hubColor = green;
            case INACTIVE -> hubColor = grey;
            case FIVE_SECOND_WARNING -> {
                boolean flashOn = ((System.currentTimeMillis() / 500) % 2) == 0;
                hubColor = flashOn ? yellow : grey;
            }
            default -> hubColor = grey; // default to grey if somehow in an undefined state
        }
        
        SmartDashboard.putString("Hub Color", hubColor.toHexString());

    }
}
