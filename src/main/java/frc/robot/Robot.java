// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

import static frc.robot.RobotContainer.swerveSubsystem;

public class Robot extends TimedRobot {

    private RobotContainer m_robotContainer;

    @Override
    public void robotInit() {
        m_robotContainer = new RobotContainer();
        SmartDashboard.putData("Field", RobotContainer.m_field);
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
        m_robotContainer.m_field.setRobotPose(RobotContainer.swerveSubsystem.getPose());

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

}
