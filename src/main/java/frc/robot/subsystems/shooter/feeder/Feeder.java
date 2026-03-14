package frc.robot.subsystems.shooter.feeder;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Robot;

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

  public SparkMax getSpark() {
    return io.getSpark();
  }

  public Command idleUntilReadyToFire() {
    return idle().until(readyToFire);
  }

  public Command f_activate() {
    return run(() -> {
      if (Robot.overrideState == Constants.OverrideState.SAFE) {
        if (readyToFire.getAsBoolean()) {
          io.setVoltage(Constants.FeederConstants.FEED_VOLTAGE);
        }
      } else {
        io.setVoltage(Constants.FeederConstants.FEED_VOLTAGE);
      }
    });
  }

  public Command f_idleThenActivate() {
    // return idleUntilReadyToFire().andThen(f_activate());
    return f_activate();
  }

  public Command o_stop() {
    return runOnce(io::stop);
  }
}
