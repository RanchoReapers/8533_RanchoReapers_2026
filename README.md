(for ex. Team 4 creates a menu to run automations in their dashboard -- can do similar) -> create auto shortcuts 
    
    public RobotContainer() {

        NamedCommands.registerCommand("align to left side", new Shift(drivetrain, m_vision, MaxSpeed, Pipeline.LEFT_PIPE));
        NamedCommands.registerCommand("align to right side", new Shift(drivetrain, m_vision, MaxSpeed, Pipeline.RIGHT_PIPE));
        NamedCommands.registerCommand("level 1", new ElevateAndPivot(m_elevator, m_pivot, Elevator.Level.LEVEL_1, 0, 0).withTimeout(1));
        NamedCommands.registerCommand("level 2", new ElevateAndPivot(m_elevator, m_pivot, Elevator.Level.LEVEL_2, 0, 0).withTimeout(1));
        NamedCommands.registerCommand("level 3", new ElevateAndPivot(m_elevator, m_pivot, Elevator.Level.LEVEL_3, 0, 0).withTimeout(1));
        NamedCommands.registerCommand("level 4", new ElevateAndPivot(m_elevator, m_pivot, Elevator.Level.LEVEL_4, 0, 0).withTimeout(1));
        NamedCommands.registerCommand("CoralStation", new ElevateAndPivot(m_elevator, m_pivot, Elevator.Level.CORAL_STATION, 0, 0).withTimeout(2));
        NamedCommands.registerCommand("outtake coral", m_lowerJaw.c_intakeCoral(-0.12).withTimeout(1.5));
        NamedCommands.registerCommand("Lower elevator", new ElevateAndPivot(m_elevator, m_pivot, Elevator.Level.LEVEL_1, 1, 0).withTimeout(1.5));
        NamedCommands.registerCommand("Intake Algae", new IntakeAlgae(m_upperJaw, m_lowerJaw, 0.7, 0.7));
        NamedCommands.registerCommand("IntakeCoral", m_lowerJaw.c_intakeCoral(JawConstants.intakeSpeed).withTimeout(1.5));
        // creates a menu on shuffle board for autons
        sendableAuton = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", sendableAuton);
        Commands.run(()->logger.telemeterize(drivetrain.getState())).ignoringDisable(true).schedule();

        configureBindings();
        ShuffleboardHelper.getInstance().initialize();
    }
