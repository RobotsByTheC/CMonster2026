// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Constants.InputConstants.CONTROLLER_PORT;
import static frc.robot.Constants.InputConstants.LEFT_JOYSTICK_PORT;
import static frc.robot.Constants.InputConstants.RIGHT_JOYSTICK_PORT;
import static frc.robot.Constants.SwerveConstants.DriveConstants.MAX_DRIVE_SPEED;
import static frc.robot.Constants.SwerveConstants.TurnConstants.MAX_TURN_SPEED;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.epilogue.logging.FileBackend;
import edu.wpi.first.epilogue.logging.NTEpilogueBackend;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.data.LookupTable;
import frc.robot.sim.SimulationContext;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.RealIntakeIO;
import frc.robot.subsystems.intake.SimIntakeIO;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.RealSwerveIO;
import frc.robot.subsystems.swerve.SimSwerveIO;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.RealHopperIO;
import frc.robot.subsystems.hopper.SimHopperIO;

import java.util.function.Supplier;

@Logged
public class Robot extends TimedRobot {
	private Command autonomousCommand;
//	private final Intake intake;
	private final Swerve swerve;
	private final Shooter shooter;
	private final Vision vision;
//	private final Hopper hopper;

	public MutDistance shooterSimDistance = Meters.mutable(1);
	private double childLockMultiplier = 1;

	@NotLogged private final CommandXboxController operatorController;
	@NotLogged private final CommandJoystick leftFlightStick;
	@NotLogged private final CommandJoystick rightFlightStick;

	public Robot() {
		if (Robot.isSimulation()) {
//			intake = new Intake(new SimIntakeIO());
			swerve = new Swerve(new SimSwerveIO());
			shooter = new Shooter(false);
//			hopper = new Hopper(new SimHopperIO());
		} else {
//			intake = new Intake(new RealIntakeIO());
			swerve = new Swerve(new RealSwerveIO());
			shooter = new Shooter(true);
//			hopper = new Hopper(new RealHopperIO());
		}

		vision = new Vision();

		DriverStation.silenceJoystickConnectionWarning(true);

		operatorController = new CommandXboxController(CONTROLLER_PORT);
		leftFlightStick = new CommandJoystick(LEFT_JOYSTICK_PORT);
		rightFlightStick = new CommandJoystick(RIGHT_JOYSTICK_PORT);

		SignalLogger.start();
		DriverStation.startDataLog(DataLogManager.getLog(), true);

		Epilogue.configure(config -> config.backend = EpilogueBackend.multi(new FileBackend(DataLogManager.getLog()),
				new NTEpilogueBackend(NetworkTableInstance.getDefault())));

//		intake.setDefaultCommand(intake.f_stowAndIdle());
		swerve.setDefaultCommand(f_driveWithFlightSticks());
//		shooter.setDefaultCommand(shooter.f_idle());
//		hopper.setDefaultCommand(hopper.f_idle());

    operatorController.x().whileTrue(shooter.synchronizedRev(() -> RotationsPerSecond.of(10)));

		operatorController.a().onTrue(
				Commands.runOnce(() -> shooterSimDistance.mut_setMagnitude(shooterSimDistance.in(Meters) + 0.1)));
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
		SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
		Epilogue.update(this);
		if (Robot.isSimulation()) {
			LookupTable.update(shooterSimDistance);
		} else {
			LookupTable.update(Meters.of(Math.hypot(vision.getRelativeTarget().getX(), vision.getRelativeTarget().getY())));
		}

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
		return MAX_DRIVE_SPEED.times(rawValue).times(childLockMultiplier);
	}
	private AngularVelocity getAngularJoystickVelocity(double rawValue) {
		return MAX_TURN_SPEED.times(rawValue).times(childLockMultiplier);
	}

	public Command f_driveWithFlightSticks() {
		return swerve.f_drive(() -> getLinearJoystickVelocity(rightFlightStick.getX()),
				() -> getLinearJoystickVelocity(rightFlightStick.getY()),
				() -> getAngularJoystickVelocity(leftFlightStick.getTwist()));
	}

	public Command f_driveLockedOn() {
		return swerve.f_driveLocked(() -> getLinearJoystickVelocity(rightFlightStick.getX()),
				() -> getLinearJoystickVelocity(rightFlightStick.getY()), vision::getRelativeTarget);
	}

	public Command f_lockOnAndRev() {
		return f_driveLockedOn().alongWith(shooter.f_aimAndRev());
	}
}
