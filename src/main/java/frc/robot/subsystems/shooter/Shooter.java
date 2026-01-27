package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.Pair;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;

import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Robot;

@Logged
public class Shooter extends SubsystemBase {
	private final ShooterIO io;
  private Distance lastDistanceToTarget = Meters.zero();

  class Flywheel extends SubsystemBase {
    public Command setTargetVelocity(AngularVelocity velocity) {
      return runOnce(() -> io.setFlywheelVelocity(velocity));
    }
    public Command stop() {
      return runOnce(io::stopFlywheel);
    }
  }

  class Hood extends SubsystemBase {
    public Command setTargetAngle(Angle angle) {
      return runOnce(() -> io.setHoodAngle(angle));
    }
    public Command stop() {
      return runOnce(io::stopHood);
    }
  }

	public Shooter(ShooterIO io) {
		this.io = io;
	}


}
