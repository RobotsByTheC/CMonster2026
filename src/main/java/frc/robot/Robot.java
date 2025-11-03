// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Feet;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.epilogue.logging.FileBackend;
import edu.wpi.first.epilogue.logging.NTEpilogueBackend;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.sim.SimulationContext;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.MAXSwerveIO;
import frc.robot.subsystems.drive.SimSwerveIO;
import java.util.function.Supplier;

@Logged
public class Robot extends TimedRobot {
  private Command autonomousCommand;
  @Logged private final CommandScheduler scheduler = CommandScheduler.getInstance();

  // The robot's subsystems
  private final DriveSubsystem drive;
  private final Vision vision;

  private final int turnMultiplier = 1;

  // Driver and operator controls
  @NotLogged private final CommandXboxController operatorController; // NOPMD
  @NotLogged private final CommandJoystick rStick; // NOPMD
  @NotLogged private final CommandJoystick lStick; // NOPMD

  @SuppressWarnings("FieldCanBeLocal")
  private final SendableChooser<Supplier<Command>> odometryTestChooser = new SendableChooser<>();

  private final SendableChooser<Supplier<Command>> autoChooser = new SendableChooser<>();

  public Robot() {
    // Initialize our subsystems. If our program is running in simulation mode (either from the
    // simulate command in vscode or from running in unit tests), then we use the simulation IO
    // layers. Otherwise, the IO layers that interact with real hardware are used.
    if (Robot.isSimulation()) {
      drive = new DriveSubsystem(new SimSwerveIO());
    } else {
      drive = new DriveSubsystem(new MAXSwerveIO());
    }
    vision = new Vision();

    odometryTestChooser.setDefaultOption("Do Nothing", Commands::none);
    odometryTestChooser.addOption("Drive 1 Foot", () -> drive.driveDistance(Feet.of(1)));
    odometryTestChooser.addOption("Drive 2 Feet", () -> drive.driveDistance(Feet.of(2)));
    odometryTestChooser.addOption("Drive 3 Feet", () -> drive.driveDistance(Feet.of(3)));
    odometryTestChooser.addOption("Drive 5 Feet", () -> drive.driveDistance(Feet.of(5)));
    odometryTestChooser.addOption("Drive 8 Feet", () -> drive.driveDistance(Feet.of(8)));
    odometryTestChooser.addOption("Drive 13 Feet", () -> drive.driveDistance(Feet.of(13)));
    odometryTestChooser.addOption("Drive 21 Feet", () -> drive.driveDistance(Feet.of(21)));

    autoChooser.setDefaultOption("Do Nothing", Commands::none);
    autoChooser.addOption("10s Leave", drive::autoLeaveArea);

    Shuffleboard.getTab("Test").add("Drive Distance Selection", odometryTestChooser);
    SmartDashboard.putData("Auto Chooser", autoChooser);

    operatorController = new CommandXboxController(Constants.OIConstants.driverControllerPort);
    rStick = new CommandJoystick(Constants.OIConstants.leftJoystickPort);
    lStick = new CommandJoystick(Constants.OIConstants.rightJoystickPort);

    // Configure the button bindings and automatic bindings

    // Configure default commands
    /*
     * Use driveWithFlightSticks() to use flight stick driving
     * Use driveWithXbox() to drive solely with the xbox controller
     * Note: Right joystick drives, left joystick turns for both
     * xbox and flight sticks. Refer to Constants.java (OIConstants)
     * for the correct Driver Station inputs.
     */
    drive.setDefaultCommand(driveFastWithFlightSticks());

    // Start data logging

    SignalLogger.start();

    Epilogue.configure(
        config ->
            config.backend =
                EpilogueBackend.multi(
                    new FileBackend(DataLogManager.getLog()),
                    new NTEpilogueBackend(NetworkTableInstance.getDefault())));

    DriverStation.startDataLog(DataLogManager.getLog(), true);

    // Disable joystick warnings by default
    DriverStation.silenceJoystickConnectionWarning(true);
  }

  @SuppressWarnings("unused")
  private Command driveWithXbox() {
    //noinspection SuspiciousNameCombination
    return drive.driveFastWithJoysticks(
        operatorController::getLeftY, operatorController::getLeftX, operatorController::getRightX);
  }

  private Command driveFastWithFlightSticks() {
    //noinspection SuspiciousNameCombination
    return drive.driveFastWithJoysticks(
        lStick::getY, lStick::getX, () -> rStick.getTwist() * turnMultiplier);
  }

  @SuppressWarnings("unused")
  private Command driveSlowWithFlightSticks() {
    //noinspection SuspiciousNameCombination
    return drive.driveSlowWithJoysticks(lStick::getY, lStick::getX, rStick::getTwist);
  }

  /**
   * Gets the command to run in autonomous based on user selection in a dashboard.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected().get();
  }

  @Logged(name = "Battery Voltage")
  public double getBatteryVoltage() {
    return RobotController.getBatteryVoltage();
  }

  @Logged(name = "Current Draw")
  public double getBatteryCurrentDraw() {
    return RobotController.getInputCurrent();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    if (DriverStation.isFMSAttached()) {
      // Enable joystick warnings on the field
      DriverStation.silenceJoystickConnectionWarning(false);
    }

    vision.update();

    Epilogue.update(this);

    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.

    CommandScheduler.getInstance().run();

    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
  }

  @Override
  public void simulationPeriodic() {
    SimulationContext.getDefault().update(getPeriod());
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by the autoChooser. */
  @Override
  public void autonomousInit() {
    autonomousCommand = getAutonomousCommand();

    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testPeriodic() {}
}
