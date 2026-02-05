package frc.robot.subsystems.shooter.feeder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static frc.robot.Constants.FeederConstants.*;

public class Feeder extends SubsystemBase {
	private final FeederIO io;
  private final Trigger readyToFire;

  private boolean isQueued = false;

	public Feeder(FeederIO io, Trigger readyToFire) {
		this.io = io;

    this.readyToFire = readyToFire;
  }

  public Command queueBall() {
    return runOnce(() -> isQueued = true).andThen(idle().until(readyToFire)).andThen(activate().until());
  }

  public Command activate() {
    return run(() -> io.setVoltage(FEED_VOLTAGE));
  }

  public Command stop() {
    return run(io::stop);
  }
}
