// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.InputConstants.CONTROLLER_PORT;
import static frc.robot.Constants.InputConstants.LEFT_JOYSTICK_PORT;
import static frc.robot.Constants.InputConstants.RIGHT_JOYSTICK_PORT;
import static frc.robot.Constants.SwerveConstants.DriveConstants.MAX_DRIVE_SPEED;
import static frc.robot.Constants.SwerveConstants.TurnConstants.MAX_TURN_SPEED;
import static frc.robot.Constants.VisionConstants.BLUE_HUB;
import static frc.robot.Constants.VisionConstants.RED_HUB;

import com.ctre.phoenix6.SignalLogger;
import com.reduxrobotics.canand.CanandEventLoop;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.epilogue.logging.FileBackend;
import edu.wpi.first.epilogue.logging.NTEpilogueBackend;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.data.LookupTable;
import frc.robot.sim.SimulationContext;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.RealSwerveIO;
import frc.robot.subsystems.swerve.SimSwerveIO;
import frc.robot.subsystems.swerve.Swerve;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import jdk.jfr.Percentage;
import frc.robot.subsystems.leds.LEDs;

@Logged
public class Robot extends TimedRobot {
  private Command autonomousCommand;
  // private final Intake intake;
  private final Swerve swerve;
  private final Shooter shooter;
  private final PoseEstimation poseEstimation;
  private final LEDs leds;
  // private final Hopper hopper;

  public MutDistance operatorFudgeFactor = Meters.mutable(0);

  @NotLogged private final CommandXboxController operatorController;
  @NotLogged private final CommandJoystick leftFlightStick;
  @NotLogged private final CommandJoystick rightFlightStick;

  public Robot() {
    if (Robot.isSimulation()) {
      // intake = new Intake(new SimIntakeIO());
      swerve = new Swerve(new SimSwerveIO());
      shooter = new Shooter(false);
      // hopper = new Hopper(new SimHopperIO());
    } else {
      // intake = new Intake(new RealIntakeIO());
      swerve = new Swerve(new RealSwerveIO());
      shooter = new Shooter(true);
      // hopper = new Hopper(new RealHopperIO());
    }

    poseEstimation = new PoseEstimation();
    leds = new LEDs();

    DriverStation.silenceJoystickConnectionWarning(true);

    operatorController = new CommandXboxController(CONTROLLER_PORT);
    leftFlightStick = new CommandJoystick(LEFT_JOYSTICK_PORT);
    rightFlightStick = new CommandJoystick(RIGHT_JOYSTICK_PORT);

    SignalLogger.start();
    DriverStation.startDataLog(DataLogManager.getLog(), true);
    CanandEventLoop.getInstance();

    Epilogue.configure(config -> config.backend = EpilogueBackend.multi(new FileBackend(DataLogManager.getLog()),
        new NTEpilogueBackend(NetworkTableInstance.getDefault())));

    // intake.setDefaultCommand(intake.f_stowAndIdle());
    swerve.setDefaultCommand(f_driveWithFlightSticks());
    // hopper.setDefaultCommand(hopper.f_idle());

    leds.setDefaultCommand(leds.runPattern(LEDPattern.solid(Color.kRed)));

    bindDriverButtons();
    bindOperatorButtons();
  }

  public void bindDriverButtons() {
    // leftFlightStick.trigger().whileTrue(f_lockOnAndRev());
  }

  public void bindOperatorButtons() {
    operatorController.x().whileTrue(shooter.f_feed());
    operatorController.leftBumper().onTrue(shooter.f_idleAtSpeed().alongWith(shooter.l_normalizeHood()));
    operatorController.rightBumper().whileTrue(shooter.f_aimAndRev());
    operatorController.rightBumper().onFalse(shooter.f_idleAtSpeed().alongWith(shooter.l_normalizeHood()));
    operatorController.povUp()
        .onTrue(Commands.runOnce(() -> operatorFudgeFactor.mut_setMagnitude(operatorFudgeFactor.magnitude() + 0.1)));
    operatorController.povDown()
        .onTrue(Commands.runOnce(() -> operatorFudgeFactor.mut_setMagnitude(operatorFudgeFactor.magnitude() - 0.1)));
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
    poseEstimation.update(swerve.getHeading(), swerve.getModulePositions());

     DriverStation.getAlliance().ifPresent((alliance -> LookupTable.update(poseEstimation
     .getDistanceToPose((alliance.equals(DriverStation.Alliance.Blue)) ? BLUE_HUB : RED_HUB).distance())));
    Epilogue.update(this);
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void simulationPeriodic() {
    SimulationContext.getDefault().update(getPeriod());
  }

  @Override
  public void autonomousInit() {
    if (autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void robotInit() {
    operatorFudgeFactor.mut_setMagnitude(0);
  }

  private LinearVelocity getLinearJoystickVelocity(double rawValue) {
    return MAX_DRIVE_SPEED.times(MathUtil.applyDeadband(rawValue, 0.1));
  }
  private AngularVelocity getAngularJoystickVelocity(double rawValue) {
    return MAX_TURN_SPEED.times(MathUtil.applyDeadband(rawValue, 0.1));
  }

  public Command f_driveWithFlightSticks() {
    return swerve.f_drive(() -> getLinearJoystickVelocity(rightFlightStick.getX() * -1),
        () -> getLinearJoystickVelocity(rightFlightStick.getY()),
        () -> getAngularJoystickVelocity(leftFlightStick.getTwist()));
  }

  // public Command f_driveLockedOn() {
  // return swerve.f_driveLocked(() -> getLinearJoystickVelocity(rightFlightStick.getX()),
  // () -> getLinearJoystickVelocity(rightFlightStick.getY()), () -> {
  // if (DriverStation.getAlliance().isPresent()) {
  // if (DriverStation.getAlliance().get().equals(DriverStation.Alliance.Blue)) {
  // return poseEstimation.getDistanceToHub(BLUE_HUB).angle();
  // } else {
  // return poseEstimation.getDistanceToHub(RED_HUB).angle();
  // }
  // }
  // return Radians.zero();
  // });
  // }

  // public Command f_lockOnAndRev() {
  // return f_driveLockedOn().alongWith(shooter.f_aimAndRev());
  // }
}
