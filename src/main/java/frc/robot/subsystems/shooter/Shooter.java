package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.Pair;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.sql.SQLOutput;
import java.util.function.Supplier;

import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Robot;

@Logged
public class Shooter extends SubsystemBase {
	private final ShooterIO io;
  private final Flywheel flywheel;
  private final Hood hood;

  private MutDistance lastDistanceToTarget = Meters.mutable(0);

  class Flywheel extends SubsystemBase {
    public Command stop() {
      return runOnce(io::stopFlywheel);
    }
  }

  class Hood extends SubsystemBase {
    public Command stop() {
      return runOnce(io::stopHood);
    }
  }

	public Shooter(ShooterIO io) {
    this.io = io;
    flywheel = new Flywheel();
    hood = new Hood();
  }

  
}
