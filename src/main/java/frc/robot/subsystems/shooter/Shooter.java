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

  private final MutDistance lastDistanceToTarget = Meters.mutable(0);

	public Shooter(ShooterIO io) {
    this.io = io;
  }

  public Command f_shootDistance(Supplier<Distance> distance) {
    return run(() -> {
      if (!lastDistanceToTarget.isNear(distance.get(), Inches.of(0.5)) || lastDistanceToTarget.magnitude() == 0) {
        io.setFlywheelVelocity(LookupTable.SPEED_TABLE.get(distance.get()));
        io.setHoodAngle(LookupTable.ANGLE_TABLE.get(distance.get()));
        lastDistanceToTarget.mut_setMagnitude(distance.get().magnitude());
      }
    });
  }

  public Command o_resetDistance() {
    return runOnce(() -> lastDistanceToTarget.mut_setMagnitude(0));
  }

  public Command f_idle() {
    return run(() -> {
      io.stopFlywheel();
      io.stopHood();
    });
  }
}
