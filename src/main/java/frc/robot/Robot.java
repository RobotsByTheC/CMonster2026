// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.InputConstants.CONTROLLER_PORT;
import static frc.robot.Constants.InputConstants.LEFT_JOYSTICK_PORT;
import static frc.robot.Constants.InputConstants.RIGHT_JOYSTICK_PORT;
import static frc.robot.Constants.SwerveConstants.DriveConstants.MAX_DRIVE_SPEED;
import static frc.robot.Constants.SwerveConstants.TurnConstants.MAX_TURN_SPEED;

import com.ctre.phoenix6.SignalLogger;
import com.reduxrobotics.canand.CanandEventLoop;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.epilogue.logging.FileBackend;
import edu.wpi.first.epilogue.logging.NTEpilogueBackend;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
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

@Logged
public class Robot extends TimedRobot {
  private Command autonomousCommand;
//  private final Intake intake;
  private final Swerve swerve;
  private final Shooter shooter;
  // private final PoseEstimation poseEstimation;
//  private final Hopper hopper;

  public MutDistance shooterSimDistance = Meters.mutable(1);
  public MutVoltage appliedVoltage = Volts.mutable(0);

  @NotLogged private final CommandXboxController operatorController;
  @NotLogged private final CommandJoystick leftFlightStick;
  @NotLogged private final CommandJoystick rightFlightStick;

  private final AddressableLED led = new AddressableLED(9);
  private final AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(34 + 23);
  @Logged
  private long lastLoopTimeµs = 0;

  public Robot() {
    if (Robot.isSimulation()) {
//      intake = new Intake(new SimIntakeIO());
      swerve = new Swerve(new SimSwerveIO());
      shooter = new Shooter(false);
//      hopper = new Hopper(new SimHopperIO());
    } else {
//      intake = new Intake(new RealIntakeIO());
      swerve = new Swerve(new RealSwerveIO());
      shooter = new Shooter(true);
//      hopper = new Hopper(new RealHopperIO());
    }

    // poseEstimation = new PoseEstimation();
    led.setLength(ledBuffer.getLength());
    led.start();
    led.setData(ledBuffer);

    DriverStation.silenceJoystickConnectionWarning(true);

    operatorController = new CommandXboxController(CONTROLLER_PORT);
    leftFlightStick = new CommandJoystick(LEFT_JOYSTICK_PORT);
    rightFlightStick = new CommandJoystick(RIGHT_JOYSTICK_PORT);

    SignalLogger.start();
    DriverStation.startDataLog(DataLogManager.getLog(), true);
    CanandEventLoop.getInstance();

    Epilogue.configure(config -> config.backend = EpilogueBackend.multi(new FileBackend(DataLogManager.getLog()),
        new NTEpilogueBackend(NetworkTableInstance.getDefault())));

//    intake.setDefaultCommand(intake.f_stowAndIdle());
    swerve.setDefaultCommand(f_driveWithFlightSticks());
//    hopper.setDefaultCommand(hopper.f_idle());

    bindDriverButtons();
    bindOperatorButtons();

    SmartDashboard.putNumber("Shooter RPM", 0); // for dashboard-side tuning

    // Dim by 1/6th because the servo power module outputs 6 volts, but the LED strips take 5 volts
    LEDPattern rslBlink =
        LEDPattern.solid(Color.kOrangeRed).atBrightness(Percent.of(83))
            .synchronizedBlink(RobotController::getRSLState);
    CommandScheduler.getInstance().schedule(
        Commands.run(() -> {
              rslBlink.applyTo(ledBuffer);
              led.setData(ledBuffer);
            }).ignoringDisable(true)
            .withName("RSL Blink")
    );
  }

  public void bindDriverButtons() {
    // leftFlightStick.trigger().whileTrue(f_lockOnAndRev());
  }

  public void bindOperatorButtons() {
    operatorController.leftBumper().whileTrue(shooter.l_kapow());
    operatorController.rightBumper().whileTrue(shooter.f_aimAndRev());
    operatorController.x().whileTrue(shooter.synchronizedRev(() -> RPM.of(SmartDashboard.getNumber("Shooter RPM", 0))));
    operatorController.y().whileTrue(shooter.feed());
    operatorController.a()
        .onTrue(Commands.runOnce(() -> shooterSimDistance.mut_setMagnitude(shooterSimDistance.magnitude() + 0.1)));
    operatorController.b()
        .onTrue(Commands.runOnce(() -> shooterSimDistance.mut_setMagnitude(shooterSimDistance.magnitude() - 0.1)));
  }

  @Override
  public void robotPeriodic() {
    long start = RobotController.getFPGATime();
    CommandScheduler.getInstance().run();
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
    // poseEstimation.update(swerve.getHeading(), swerve.getModulePositions());
    // if (Robot.isSimulation()) {
    // LookupTable.update(shooterSimDistance);
    // } else {
    // DriverStation.getAlliance().ifPresent((alliance -> LookupTable.update(poseEstimation
    // .getDistanceToHub((alliance.equals(DriverStation.Alliance.Blue)) ? BLUE_HUB : RED_HUB).distance())));
    // }
    LookupTable.update(shooterSimDistance);
    Epilogue.update(this);
    lastLoopTimeµs = RobotController.getFPGATime() - start;
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

  private LinearVelocity getLinearJoystickVelocity(double rawValue) {
    return MAX_DRIVE_SPEED.times(rawValue);
  }
  private AngularVelocity getAngularJoystickVelocity(double rawValue) {
    return MAX_TURN_SPEED.times(rawValue);
  }

  public Command f_driveWithFlightSticks() {
    return swerve.f_drive(() -> getLinearJoystickVelocity(rightFlightStick.getX()),
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

  public Command f_shootBall() {
    return shooter.l_kapow().repeatedly();
  }
}
