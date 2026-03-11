package frc.robot.subsystems.shooter.feeder;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

@Logged
public class Feeder extends SubsystemBase {
  private final FeederIO io;
  private final Trigger readyToFire;

  public Feeder(FeederIO io, Trigger readyToFire) {
    this.io = io;
    this.readyToFire = readyToFire.and(io::isBallReadyToFire);
  }

  public boolean canShoot() {
    return io.isBallReadyToFire();
  }

  public Command idleUntilReadyToFire() {
    return idle().until(readyToFire);
  }

  public Command queueBall() {
    return idleUntilReadyToFire().andThen(f_activate().until(io::isBallAtFlywheel))
        .andThen(f_activate().until(() -> !io.isBallAtFlywheel())).andThen(o_stop());
  }

  public Command f_activate() {
    return run(() -> io.setVoltage(Constants.FeederConstants.FEED_VOLTAGE));
  }

  public Command o_stop() {
    return runOnce(io::stop);
  }
}
