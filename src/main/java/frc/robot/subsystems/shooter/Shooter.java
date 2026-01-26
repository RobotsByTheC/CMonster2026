package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.Pair;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;

import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Robot;

@Logged
public class Shooter extends SubsystemBase {
	private final ShooterIO io;
	public Distance lastDistanceToTarget = Meters.zero();

	public Shooter(ShooterIO io) {
		this.io = io;
	}

	private void updateTarget(Distance distanceToTarget) {
		if (distanceToTarget.isNear(lastDistanceToTarget, Inches.of(2))) return;
		io.setFlywheelVelocity(LookupTable.SPEED_TABLE.get(distanceToTarget));
		io.setHoodAngle(LookupTable.ANGLE_TABLE.get(distanceToTarget));
		lastDistanceToTarget = distanceToTarget;
	}

  private void simUpdateTarget(Distance distanceToTarget) {
    io.setFlywheelVelocity(LookupTable.SPEED_TABLE.get(distanceToTarget));
    io.setHoodAngle(LookupTable.ANGLE_TABLE.get(distanceToTarget));
    System.out.println(Math.random() + "| " + distanceToTarget + LookupTable.SPEED_TABLE.get(distanceToTarget));
  }

	public Command f_shootAtTarget(Supplier<Distance> distance) {
		return run(() -> {
      if (Robot.isSimulation()) {
        simUpdateTarget(distance.get());
      } else {
        updateTarget(distance.get());
      }
    });
	}

  public Command f_holdPos() {
    return run(() -> {
      io.setFlywheelVelocity(RPM.zero());
      io.setHoodAngle(Degrees.zero());
    });
  }

	public Command o_stop() {
		return runOnce(io::stopFlywheel).andThen(run(io::stopHood));
	}
}
