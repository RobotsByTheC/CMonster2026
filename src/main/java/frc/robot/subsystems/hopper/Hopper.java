package frc.robot.subsystems.hopper;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

@Logged
public class Hopper extends SubsystemBase {
  private final HopperIO io;

  public Hopper(HopperIO io) {
    this.io = io;
  }

  public Command f_hopperIntake() {
    return run(() -> io.setVoltage(Constants.HopperConstants.INTAKE_VOLTAGE));
  }

  public Command f_hopperReverse() {
    return run(() -> io.setVoltage(Constants.HopperConstants.OUTTAKE_VOLTAGE));
  }

  public Command f_idle() {
    return run(io::stop);
  }
}
