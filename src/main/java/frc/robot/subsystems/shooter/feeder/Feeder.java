package frc.robot.subsystems.shooter.feeder;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

import static edu.wpi.first.units.Units.Milliseconds;

@Logged
public class Feeder extends SubsystemBase {
  private final FeederIO io;
  private final Trigger readyToFire;

  public Feeder(FeederIO io, Trigger readyToFire) {
    this.io = io;
    this.readyToFire = readyToFire;
  }

  public boolean canShoot() {
    return io.isBallReadyToFire();
  }

  public Command queueBall() {
    return idle().until(readyToFire).andThen(f_activate().withTimeout(Milliseconds.of(100))).andThen(o_stop());
  }

  public Command f_activate() {
    return run(() -> io.setVoltage(Constants.MatchConstants.FEEDER_APPLY_VOLTAGE));
  }

  public Command o_stop() {
    return runOnce(io::stop);
  }
}
