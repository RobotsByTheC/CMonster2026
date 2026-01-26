// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;
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
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.sim.SimulationContext;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.RealIntakeIO;
import frc.robot.subsystems.intake.SimIntakeIO;
import frc.robot.subsystems.shooter.RealShooterIO;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.SimShooterIO;
import frc.robot.subsystems.swerve.RealSwerveIO;
import frc.robot.subsystems.swerve.SimSwerveIO;
import frc.robot.subsystems.swerve.Swerve;

@Logged
public class Robot extends TimedRobot {
	private Command autonomousCommand;
	private final Intake intake;
	private final Shooter shooter;
	private final Swerve swerve;

	@NotLogged private final CommandXboxController operatorController;
	@NotLogged private final CommandJoystick leftFlightStick;
	@NotLogged private final CommandJoystick rightFlightStick;

	public Robot() {
		if (Robot.isSimulation()) {
			intake = new Intake(new SimIntakeIO());
			shooter = new Shooter(new SimShooterIO());
			swerve = new Swerve(new SimSwerveIO());
		} else {
			intake = new Intake(new RealIntakeIO());
			shooter = new Shooter(new RealShooterIO());
			swerve = new Swerve(new RealSwerveIO());
		}

		DriverStation.silenceJoystickConnectionWarning(true);

		operatorController = new CommandXboxController(CONTROLLER_PORT);
		leftFlightStick = new CommandJoystick(LEFT_JOYSTICK_PORT);
		rightFlightStick = new CommandJoystick(RIGHT_JOYSTICK_PORT);

		SignalLogger.start();
		DriverStation.startDataLog(DataLogManager.getLog(), true);

		Epilogue.configure(config -> config.backend = EpilogueBackend.multi(new FileBackend(DataLogManager.getLog()),
				new NTEpilogueBackend(NetworkTableInstance.getDefault())));

		intake.setDefaultCommand(intake.f_stowAndIdle());
		shooter.setDefaultCommand(shooter.stop());
		swerve.setDefaultCommand(f_driveWithFlightSticks());

		operatorController.x().whileTrue(intake.f_extendAndGrab());
		operatorController.x().onFalse(intake.l_retractAndGrab());

		operatorController.y().whileTrue(shooter.f_shootAtTarget(() -> Meters.of(1)));
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
		SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
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

	public Command f_driveWithFlightSticks() {
		return swerve.f_drive(() -> MAX_DRIVE_SPEED.times(rightFlightStick.getX()),
				() -> MAX_DRIVE_SPEED.times(rightFlightStick.getY()),
				() -> MAX_TURN_SPEED.times(leftFlightStick.getTwist()));
	}
}
