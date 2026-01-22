// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.SimShooterIO;

@Logged
public class Robot extends TimedRobot {
  final Shooter shooter;
  final SimShooterIO simShooterIO;

  private Command m_autonomousCommand;

  public Robot() {
    if (isSimulation()) {
       simShooterIO = new SimShooterIO();
       shooter = new Shooter(simShooterIO);
     } else {
       // Running on real hardware. We haven't made an IO for that yet
       shooter = null;
       simShooterIO = null;
     }
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    Epilogue.update(this);
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = shooter.runAtSpeed(() -> RadiansPerSecond.of(800));
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {
    if (simShooterIO != null) {
      simShooterIO.updateSim();
    }
  }
}
