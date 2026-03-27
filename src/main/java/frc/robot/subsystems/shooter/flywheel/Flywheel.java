package frc.robot.subsystems.shooter.flywheel;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Robot;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

@Logged
public class Flywheel extends SubsystemBase {
  private final FlywheelIO io;

  public Flywheel(FlywheelIO io) {
    this.io = io;
    this.setDefaultCommand(o_stop());
  }

  public SparkMax getLeaderMotor() {
    return io.getLeader();
  }

  public SparkMax getFollowerMotor() {
    return io.getFollower();
  }

  @NotLogged
  public boolean atTargetSpeed() {
    return io.atTargetVelocity();
  }

  public Command o_stop() {
    return runOnce(() -> io.setVoltage(Volts.zero()));
  }

  public Command f_shoot(Supplier<AngularVelocity> target) {
    return run(() -> {
      switch (Robot.shooterState) {
        case IDLE -> io.setVelocity(Constants.ShooterConstants.FlywheelConstants.IDLE_SPEED);
        case STOP -> io.setVoltage(Volts.zero());
        case TARGET -> io.setVelocity(target.get());
        case FERRY -> io.setVelocity(Constants.ShooterConstants.FlywheelConstants.FERRY_SPEED);
      }
    });
  }

  public Command f_idle() {
    return o_stop().andThen(idle());
  }

  public Command f_idleAtSpeed() {
    return f_shoot(() -> Constants.ShooterConstants.FlywheelConstants.IDLE_SPEED);
  }

  public double getPercentageSpeed() {
    if (io.getPrimaryVelocity().isNear(RPM.zero(), RPM.of(0.001)))
      return 0;
    if (io.getTargetVelocity().equals(RPM.zero()))
      return 0;
    return Math.round(io.getPrimaryVelocity().div(io.getTargetVelocity()).in(Percent) * 10) / 10d;
  }
}
