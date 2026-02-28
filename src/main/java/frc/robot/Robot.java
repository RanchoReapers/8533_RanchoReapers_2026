// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static frc.robot.RobotContainer.intakeSubsystem;
// import static frc.robot.RobotContainer.shooterSubsystem;
import static frc.robot.RobotContainer.limelightDetectionSubsystem;
// UNCOMMENT THESE WHEN ROBOT IS BUILT AND WIRED
import static frc.robot.RobotContainer.swerveSubsystem;

import java.util.Optional;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {

    private final Alert hubStatus = new Alert("Please connect to FMS to use Alliance Switching Alerts", AlertType.kWarning);
    private final Alert corruptFMSData = new Alert("FMS Data is corrupt; alliance switching alerts CANNOT BE TRUSTED.", AlertType.kError);
    private RobotContainer m_robotContainer;
    public final Field2d m_field = new Field2d();
    public String gameData;

    public AddressableLED m_led;
    public AddressableLEDBuffer m_ledBuffer;

    private enum HubState {
        ACTIVE, INACTIVE, FIVE_SECOND_WARNING
    }

    private HubState hubState = HubState.INACTIVE;

    private static final Distance kLedSpacing = Meters.of(1.0 / 60.0);

    private final LEDPattern m_rainbow = LEDPattern.rainbow(255, 255);
    private final LEDPattern m_fastRainbow = m_rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(1.2), kLedSpacing);
    private final LEDPattern m_slowRainbow = m_rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(0.6), kLedSpacing);

    private final LEDPattern m_purple = LEDPattern.solid(new Color(195, 0, 255));
    private final LEDPattern m_white = LEDPattern.solid(Color.kWhite);
    private final LEDPattern m_yellow = LEDPattern.solid(Color.kYellow);
    private final LEDPattern m_electricGreen = LEDPattern.solid(new Color(0, 255, 72));
    private final LEDPattern m_wine = LEDPattern.solid(new Color(135, 0, 88));
    private final LEDPattern m_blue = LEDPattern.solid(Color.kBlue);
    private final LEDPattern m_red = LEDPattern.solid(Color.kRed);
    private final LEDPattern m_orange = LEDPattern.solid(new Color(255, 145, 0));

    @Override
    public void robotInit() {
        m_robotContainer = new RobotContainer();

        SmartDashboard.putData("Field", m_field);

        SmartDashboard.putData(CommandScheduler.getInstance());

        m_led = new AddressableLED(9); // PWM port 9
        m_ledBuffer = new AddressableLEDBuffer(150); // 150 LEDs
        m_led.setLength(m_ledBuffer.getLength());
        m_led.setData(m_ledBuffer);
        m_led.start();
    }

    @Override
    public void disabledPeriodic() {
        if (m_led != null && m_ledBuffer != null) {

            Optional<Alliance> ally = DriverStation.getAlliance();

            if (DriverStation.isFMSAttached() && ally.isPresent() && (ally.get() == Alliance.Blue)) {
                applySolidColorPattern(m_blue);
            } else if (DriverStation.isFMSAttached() && ally.isPresent() && (ally.get() == Alliance.Red)) {
                applySolidColorPattern(m_red);
            } else if (DriverStation.isFMSAttached()) {
                applySolidColorPattern(m_orange);
            } else {
                applySolidColorPattern(m_purple);
            }

        }
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

        if (DriverStation.isFMSAttached() && matchTime >= 28.0 && matchTime <= 33.0) {
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
    }

    @Override
    public void autonomousPeriodic() {
        swerveSubsystem.periodic();
        applyRainbowCycle("slow");
    }

    @Override
    public void teleopInit() {
        intakeSubsystem.intakeMotorStopped = true;
        // shooterSubsystem.shooterMotorStopped = true;
        // UNCOMMENT THESE WHEN ROBOT IS BUILT AND WIRED

        hubStatus.set(false);
        corruptFMSData.set(false);
        limelightDetectionSubsystem.setPipelineIndex(0);
    }

    @Override
    public void autonomousInit() {
        limelightDetectionSubsystem.setPipelineIndex(1);
    }

    @Override
    public void teleopPeriodic() {
        if (DriverStation.isFMSAttached()) {
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
                                default ->
                                    corruptFMSData.set(true);
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
                                default ->
                                    corruptFMSData.set(true);
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
                                default ->
                                    corruptFMSData.set(true);
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
                                default ->
                                    corruptFMSData.set(true);
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
                                default ->
                                    corruptFMSData.set(true);
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
                                default ->
                                    corruptFMSData.set(true);
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

            Color purple = new Color(195, 0, 255);

            // Update addressable LEDs
            if (m_led != null && m_ledBuffer != null) {

                double matchTime = DriverStation.getMatchTime();

                if (limelightDetectionSubsystem.getAimAssistActive() && getNoAimAssistInterference()) { // aim assist active with no interference: solid cyan
                    applySolidColorPattern(m_electricGreen);
                } else if (limelightDetectionSubsystem.getAimAssistActive() && !getNoAimAssistInterference()) { // aim assist active with interference: flashing cyan/red
                    applyFlashing(m_electricGreen, m_wine, 300);
                } else if (matchTime > 0 && matchTime <= 30.0) { // Endgame: show a colorful rainbow sweep to celebrate / indicate urgency
                    if (matchTime <= 5) {
                        applyRainbowCycle("fast"); // speed factor
                    } else {
                        applyRainbowCycle("slow"); // speed factor
                    }
                } else { // Active: green pulse; Five Second Warning: flash between yellow and gray; default: gray
                    switch (hubState) {
                        case ACTIVE ->
                            applyPulse(purple, 1.2);
                        case FIVE_SECOND_WARNING ->
                            applyFlashing(m_yellow, m_white, 250);
                        default ->
                            applySolidColorPattern(m_white);
                    }
                }
            }
        } else {
            if (RobotController.getRSLState()) {
                applySolidColorPattern(m_yellow);
            } else if (!RobotController.getRSLState()) {
                applySolidColorPattern(m_purple);
            }
        }

    }

    private boolean getNoAimAssistInterference() {

        boolean foughtAgainstX = true;
        boolean foughtAgainstY = true;
        boolean foughtAgainstTurning = true;

        if (limelightDetectionSubsystem.getXSpeedLimelight() == 0) {
            foughtAgainstX = false;
        }

        if (limelightDetectionSubsystem.getXSpeedLimelight() != 0 && limelightDetectionSubsystem.xSpeedBeforeLimelight == 0) {
            foughtAgainstX = false;
        }

        if (limelightDetectionSubsystem.getYSpeedLimelight() == 0) {
            foughtAgainstY = false;
        }

        if (limelightDetectionSubsystem.getYSpeedLimelight() != 0 && limelightDetectionSubsystem.ySpeedBeforeLimelight == 0) {
            foughtAgainstY = false;
        }

        if (limelightDetectionSubsystem.getTurnSpeedLimelight() == 0) {
            foughtAgainstTurning = false;
        }

        if (limelightDetectionSubsystem.getTurnSpeedLimelight() != 0 && limelightDetectionSubsystem.turningSpeedBeforeLimelight == 0) {
            foughtAgainstTurning = false;
        }

        return !(foughtAgainstX || foughtAgainstY || foughtAgainstTurning);

    }

    // ---------- LED helper methods ----------
    private void applySolidColorPattern(LEDPattern pattern) {
        pattern.applyTo(m_ledBuffer);
        m_led.setData(m_ledBuffer);
    }

    private void applyFlashing(LEDPattern onPattern, LEDPattern offPattern, long periodMs) {
        boolean on = ((System.currentTimeMillis() / periodMs) % 2) == 0;
        LEDPattern p = on ? onPattern : offPattern;
        applySolidColorPattern(p);
    }

    private void applyPulse(Color baseColor, double periodSeconds) {
        double phase = (System.currentTimeMillis() % (long) (periodSeconds * 1000)) / (periodSeconds * 1000);
        // pulsate between 40% and 100% brightness
        double brightness = 0.4 + 0.6 * 0.5 * (1 + Math.sin(2 * Math.PI * phase));
        int r = (int) (clamp(baseColor.red) * brightness * 255);
        int g = (int) (clamp(baseColor.green) * brightness * 255);
        int b = (int) (clamp(baseColor.blue) * brightness * 255);
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, r, g, b);
        }
        m_led.setData(m_ledBuffer);
    }

    private void applyRainbowCycle(String speedFactor) {
        if (speedFactor.equals("fast")) {
            m_fastRainbow.applyTo(m_ledBuffer);
            m_led.setData(m_ledBuffer);
        } else if (speedFactor.equals("slow")) {
            m_slowRainbow.applyTo(m_ledBuffer);
            m_led.setData(m_ledBuffer);
        }
    }

    private double clamp(double v) {
        return Math.min(1.0, Math.max(0.0, v));
    }
    // ---------- end LED helpers ----------
}
