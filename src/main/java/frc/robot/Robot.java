// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.epilogue.logging.FileBackend;
import edu.wpi.first.epilogue.logging.NTEpilogueBackend;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Alert;
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
import frc.robot.logging.Issue;
import frc.robot.logging.IssueTracker;
import frc.robot.sim.SimulationContext;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.MAXSwerveIO;
import frc.robot.subsystems.drive.SimSwerveIO;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

@Logged
public class Robot extends TimedRobot {
  public final IssueTracker issueTracker = new IssueTracker();
  private final DriveSubsystem drive;
  private final Vision vision;

  @NotLogged private final CommandXboxController operatorController;
  @NotLogged private final CommandJoystick rStick;
  @NotLogged private final CommandJoystick lStick;

  private double globalTurnSpeedMultiplier = 1;
  private double globalDriveSpeedMultiplier = 1;

  private final double offsetRotationPOV = 0;
  //originally off by 30 degrees. fixed.
  private final double top = 0 + offsetRotationPOV;
  private final double bottom = 90 + offsetRotationPOV;
  private final double left = 180 + offsetRotationPOV;
  private final double right = 270 + offsetRotationPOV;

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

    //makes it so the heading always starts at zero
      //while we do start at zero, degrees add up after 360. can get to 100000s of degrees. fix
      drive.resetHeading();
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

      //This chunk makes it so the robot snaps to different headings in accordance to the left sticks' POV (circle thing on the top)
      //down and left are swapped. Weirdly, it works. fix later!!!
      lStick.povUp().onTrue(drive.rotateToHeading(new Rotation2d(Degrees.of(top))));
      lStick.povRight().onTrue(drive.rotateToHeading(new Rotation2d(Degrees.of(right))));
      lStick.povDown().onTrue(drive.rotateToHeading(new Rotation2d(Degrees.of(left))));
      lStick.povLeft().onTrue(drive.rotateToHeading(new Rotation2d(Degrees.of(bottom))));

      //For debugging
      //drives forward 30
//      rStick.button(12).onTrue(drive.driveDistance(Feet.of(30)));
      //drives backward 30
//      rStick.button(11).onTrue(drive.driveDistance(Feet.of(-30)));

    operatorController
        .x()
        .onTrue(
            drive.myThirdAttemptAtDriveToRobotRelativePose(
                () -> vision.getTargetPose().toPose2d()));
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

  //Attempted to log the inputs from X and Y on the driving controllers.
    //Unable to log, shows up as 0. fix later
    //This was to get more info about the robot drifting slightly while moving. Concluded it must be -
    // - driver input becuase the robot driving autonomusly moved in a straight line.
    @Logged double x_joy;
    @Logged double y_joy;
    @Logged double twist_joy;
  // endregion
  // region | COMMANDS |
  private Command driveWithFlightSticks() {
      x_joy = rStick.getX();
      y_joy = rStick.getY();
      twist_joy = rStick.getTwist();
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

    IssueTracker.periodicUpdate();
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

//    System.out.println(drive.getHeading());
  }
  // endregion
}
