// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import static frc.robot.RobotContainer.intakeSubsystem;
// import static frc.robot.RobotContainer.shooterSubsystem;
// UNCOMMENT THESE WHEN ROBOT IS BUILT AND WIRED
import static frc.robot.RobotContainer.swerveSubsystem;

import java.util.Optional;

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
        LEDPattern allyColorFMSPresent = LEDPattern.solid(Color.kDeepPink);

        Optional<Alliance> ally = DriverStation.getAlliance();

        if (DriverStation.isFMSAttached() && ally.isPresent() && (ally.get() == Alliance.Blue)) {
            allyColorFMSPresent = LEDPattern.solid(Color.kBlue);
        } else if (DriverStation.isFMSAttached() && ally.isPresent() && (ally.get() == Alliance.Red)) {
            allyColorFMSPresent = LEDPattern.solid(Color.kRed);
        }

        // If FMS attached, solid alliance color. If not, display a moving "chase" to draw attention.
        if (m_led != null && m_ledBuffer != null) {
            if (DriverStation.isFMSAttached()) {
                allyColorFMSPresent.applyTo(m_ledBuffer);
                m_led.setData(m_ledBuffer);
            } else {
                // fun chase with med purple and black background
                applyChase(Color.kMediumPurple, Color.kBlack, 0.12);
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

        if (DriverStation.isFMSAttached()) {
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

            // Publish hub color: green when ACTIVE, grey when INACTIVE, flashing yellow during FIVE_SECOND_WARNING
            Color green = new Color(0, 255, 0);
            Color grey = new Color(128, 128, 128);
            Color yellow = new Color(255, 255, 0);

            Color hubColor;

            switch (hubState) {
                case ACTIVE ->
                    hubColor = green;
                case INACTIVE ->
                    hubColor = grey;
                case FIVE_SECOND_WARNING -> {
                    boolean flashOn = ((System.currentTimeMillis() / 500) % 2) == 0;
                    hubColor = flashOn ? yellow : grey;
                }
                default ->
                    hubColor = grey; // default to grey if somehow in an undefined state
            }

            SmartDashboard.putString("Hub Color", hubColor.toHexString());

            // Update addressable LEDs
            if (m_led != null && m_ledBuffer != null) {
                double matchTime = DriverStation.getMatchTime();

                // Endgame: show a colorful rainbow sweep to celebrate / indicate urgency
                if (matchTime > 0 && matchTime <= 30.0) {
                    if (matchTime <= 5) {
                        applyRainbowCycle(1.2); // speed factor) {
                    } else {
                        applyRainbowCycle(0.6); // speed factor
                    }

                } else {
                    // Active: green pulse; Five Second Warning: flash between yellow and gray; default: gray
                    switch (hubState) {
                        case ACTIVE ->
                            applyPulse(hubColor, 1.2);
                        case FIVE_SECOND_WARNING ->
                            applyFlashing(Color.kYellow, Color.kGray, 250);
                        default ->
                            applySolidColor(hubColor);
                    }
                }
            }
        }
    }

    // ---------- LED helper methods ----------
    private void applySolidColor(Color c) {
        LEDPattern.solid(c).applyTo(m_ledBuffer);
        m_led.setData(m_ledBuffer);
    }

    private void applyFlashing(Color onColor, Color offColor, long periodMs) {
        boolean on = ((System.currentTimeMillis() / periodMs) % 2) == 0;
        Color c = on ? onColor : offColor;
        applySolidColor(c);
    }

    private void applyChase(Color dotColor, Color bgColor, double speedSecondsPerLoop) {
        int len = m_ledBuffer.getLength();
        long t = System.currentTimeMillis();
        int pos = (int) ((t / (long) (speedSecondsPerLoop * 1000)) % len);
        int rBg = (int) (bgColor.red * 255);
        int gBg = (int) (bgColor.green * 255);
        int bBg = (int) (bgColor.blue * 255);
        int rDot = (int) (dotColor.red * 255);
        int gDot = (int) (dotColor.green * 255);
        int bDot = (int) (dotColor.blue * 255);
        for (int i = 0; i < len; i++) {
            if (i == pos) {
                m_ledBuffer.setRGB(i, rDot, gDot, bDot);
            } else {
                m_ledBuffer.setRGB(i, rBg, gBg, bBg);
            }
        }
        m_led.setData(m_ledBuffer);
    }

    private void applyPulse(Color baseColor, double periodSeconds) {
        double phase = (System.currentTimeMillis() % (long) (periodSeconds * 1000)) / (periodSeconds * 1000);
        // pulsate between 40% and 100% brightness
        double brightness = 0.4 + 0.6 * (0.5 * (1 + Math.sin(2 * Math.PI * phase)));
        int r = (int) (clamp01(baseColor.red) * brightness * 255);
        int g = (int) (clamp01(baseColor.green) * brightness * 255);
        int b = (int) (clamp01(baseColor.blue) * brightness * 255);
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, r, g, b);
        }
        m_led.setData(m_ledBuffer);
    }

    private void applyRainbowCycle(double speedFactor) {
        int len = m_ledBuffer.getLength();
        long t = System.currentTimeMillis();
        // speedFactor controls how fast it cycles; tweak as desired
        double offset = (t / 1000.0) * speedFactor;
        for (int i = 0; i < len; i++) {
            double hue = (360.0 * ((i / (double) len) + offset)) % 360.0;
            int[] rgb = hsvToRgb((float) hue, 1.0f, 1.0f);
            m_ledBuffer.setRGB(i, rgb[0], rgb[1], rgb[2]);
        }
        m_led.setData(m_ledBuffer);
    }

    private int[] hsvToRgb(float h, float s, float v) {
        float c = v * s;
        float x = c * (1 - Math.abs((h / 60.0f) % 2 - 1));
        float m = v - c;
        float r1 = 0, g1 = 0, b1 = 0;
        if (h < 60) {
            r1 = c;
            g1 = x;
            b1 = 0;
        } else if (h < 120) {
            r1 = x;
            g1 = c;
            b1 = 0;
        } else if (h < 180) {
            r1 = 0;
            g1 = c;
            b1 = x;
        } else if (h < 240) {
            r1 = 0;
            g1 = x;
            b1 = c;
        } else if (h < 300) {
            r1 = x;
            g1 = 0;
            b1 = c;
        } else {
            r1 = c;
            g1 = 0;
            b1 = x;
        }
        int r = (int) ((r1 + m) * 255);
        int g = (int) ((g1 + m) * 255);
        int b = (int) ((b1 + m) * 255);
        return new int[]{clampInt(r, 0, 255), clampInt(g, 0, 255), clampInt(b, 0, 255)};
    }

    private double clamp01(double v) {
        return Math.min(1.0, Math.max(0.0, v));
    }

    private int clampInt(int v, int lo, int hi) {
        if (v < lo) {
            return lo;
        }
        if (v > hi) {
            return hi;
        }
        return v;
    }
    // ---------- end LED helpers ----------
}
