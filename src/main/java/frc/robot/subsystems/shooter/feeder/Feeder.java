package frc.robot.subsystems.shooter.feeder;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.data.SignalDebouncer;

@Logged
public class Feeder extends SubsystemBase {
  private final FeederIO io;
  private final Trigger readyToFire;
  @NotLogged private final Trigger inputTrigger;
  private final SignalDebouncer debouncer;

  public Feeder(FeederIO io, Trigger readyToFire) {
    this.io = io;
    debouncer = new SignalDebouncer(true, 5);
    inputTrigger = readyToFire;
    this.readyToFire = new Trigger(debouncer::getResult);
  }

  @Override
  public void periodic() {
    debouncer.addValue(readyToFire.getAsBoolean());
  }

  public boolean canShoot() {
    return io.isBallReadyToFire();
  }

  public SparkMax getSpark() {
    return io.getSpark();
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

  public Command o_stop() {
    return runOnce(io::stop);
  }
}
