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
  private final DriveSubsystem drive;
  private final Vision vision;

  @NotLogged private final CommandXboxController operatorController;
  @NotLogged private final CommandJoystick rStick;
  @NotLogged private final CommandJoystick lStick;

  private double globalTurnSpeedMultiplier = 1;
  private double globalDriveSpeedMultiplier = 1;

  private final SendableChooser<Supplier<Command>> odometryTestChooser;
  private final SendableChooser<Supplier<Command>> autoChooser;

  public Robot() {
    if (Robot.isSimulation()) {
      drive = new DriveSubsystem(new SimSwerveIO());
    } else {
      drive = new DriveSubsystem(new MAXSwerveIO());
    }
    vision = new Vision();

    operatorController = new CommandXboxController(Constants.OIConstants.driverControllerPort);
    rStick = new CommandJoystick(Constants.OIConstants.rightJoystickPort);
    lStick = new CommandJoystick(Constants.OIConstants.leftJoystickPort);

    odometryTestChooser = new SendableChooser<>();
    autoChooser = new SendableChooser<>();

    configureDriveDistanceChooser();
    configureAutonomous();
    configureButtonBindings();
    configureDefaultCommands();
    startLogging();

    DriverStation.silenceJoystickConnectionWarning(true);
  }

  // region | CONFIGURE METHODS |
  private void configureDriveDistanceChooser() {
    odometryTestChooser.setDefaultOption("Do Nothing", Commands::none);
    odometryTestChooser.addOption("Drive 1 Foot", () -> drive.driveDistance(Feet.of(1)));
    odometryTestChooser.addOption("Drive 2 Feet", () -> drive.driveDistance(Feet.of(2)));
    odometryTestChooser.addOption("Drive 3 Feet", () -> drive.driveDistance(Feet.of(3)));
    odometryTestChooser.addOption("Drive 5 Feet", () -> drive.driveDistance(Feet.of(5)));
    odometryTestChooser.addOption("Drive 8 Feet", () -> drive.driveDistance(Feet.of(8)));
    odometryTestChooser.addOption("Drive 13 Feet", () -> drive.driveDistance(Feet.of(13)));
    odometryTestChooser.addOption("Drive 21 Feet", () -> drive.driveDistance(Feet.of(21)));

    Shuffleboard.getTab("Test").add("Drive Distance Selection", odometryTestChooser);
  }

  private void configureButtonBindings() {
    operatorController.y().onTrue(Commands.runOnce(() -> System.out.println("hello")));
  }

  private void configureDefaultCommands() {
    drive.setDefaultCommand(driveWithFlightSticks());
  }

  private void configureAutonomous() {
    autoChooser.setDefaultOption("Do Nothing", Commands::none);
    autoChooser.addOption("10s Leave", drive::autoLeaveArea);
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void startLogging() {
    SignalLogger.start();

    Epilogue.configure(
        config ->
            config.backend =
                EpilogueBackend.multi(
                    new FileBackend(DataLogManager.getLog()),
                    new NTEpilogueBackend(NetworkTableInstance.getDefault())));

    DriverStation.startDataLog(DataLogManager.getLog(), true);
  }

  // endregion
  // region | COMMANDS |
  private Command driveWithFlightSticks() {
    return drive.driveXYTheta(
        () -> rStick.getY() * globalDriveSpeedMultiplier,
        () -> rStick.getX() * globalDriveSpeedMultiplier,
        () -> lStick.getTwist() * globalTurnSpeedMultiplier);
  }

  // endregion
  // region | INFO METHODS |
  @Logged(name = "Battery Voltage")
  public double getBatteryVoltage() {
    return RobotController.getBatteryVoltage();
  }

  @Logged(name = "Current Draw")
  public double getBatteryCurrentDraw() {
    return RobotController.getInputCurrent();
  }

  // endregion
  // region | ROBOT OVERRIDES |
  // Runs after specific periodic functions
  @Override
  public void robotPeriodic() {
    if (DriverStation.isFMSAttached()) {
      DriverStation.silenceJoystickConnectionWarning(false);
    }

    vision.update();
    Epilogue.update(this);
    CommandScheduler.getInstance().run();
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
  }

  @Override
  public void simulationPeriodic() {
    SimulationContext.getDefault().update(getPeriod());
  }

  @Override
  public void autonomousInit() {
    Command autonomousCommand = autoChooser.getSelected().get();

    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  @Override
  public void teleopPeriodic() {
    globalTurnSpeedMultiplier = 1 - (lStick.getThrottle() + 1) / 2;
    globalDriveSpeedMultiplier = 1 - (rStick.getThrottle() + 1) / 2;
  }
  // endregion
}
