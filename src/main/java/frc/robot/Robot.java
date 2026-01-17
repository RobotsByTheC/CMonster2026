// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.InputConstants.CONTROLLER_PORT;

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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.sim.SimulationContext;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.RealIntakeIO;
import frc.robot.subsystems.intake.SimIntakeIO;

@Logged
public class Robot extends TimedRobot {
  private Command autonomousCommand;

  private final Intake intake;

  @NotLogged private final CommandXboxController operatorController;

  public Robot() {
    if (Robot.isSimulation()) {
      intake = new Intake(new SimIntakeIO());
    } else {
      intake = new Intake(new RealIntakeIO());
    }

    DriverStation.silenceJoystickConnectionWarning(true);

    operatorController = new CommandXboxController(CONTROLLER_PORT);

    SignalLogger.start();
    DriverStation.startDataLog(DataLogManager.getLog(), true);

    Epilogue.configure(
        config ->
            config.backend =
                EpilogueBackend.multi(
                    new FileBackend(DataLogManager.getLog()),
                    new NTEpilogueBackend(NetworkTableInstance.getDefault())));

    intake.setDefaultCommand(intake.c_idle());

    operatorController.x().whileTrue(intake.extendAndIntake());
    operatorController.x().onFalse(intake.retract());
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
}
