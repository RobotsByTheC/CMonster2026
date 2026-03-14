package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.Constants.HopperConstants.INTAKE_VOLTAGE;
import static frc.robot.Constants.HopperConstants.OUTTAKE_VOLTAGE;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

@Logged
public class Hopper extends SubsystemBase {
  private final HopperIO io;

  public Hopper(HopperIO io) {
    this.io = io;
  }

  public Command f_hopperIntake() {
    return Commands.repeatingSequence(
        // Run forward to pull balls towards the feeder
        run(() -> io.setVoltage(INTAKE_VOLTAGE)).withTimeout(Seconds.of(0.75)),
        // Briefly run in reverse to try to clear jams
        run(() -> io.setVoltage(OUTTAKE_VOLTAGE)).withTimeout(Seconds.of(0.25))
    ).withName("Hopper Intake");
  }

  public Command f_hopperReverse() {
    return run(() -> io.setVoltage(Constants.HopperConstants.OUTTAKE_VOLTAGE));
  }

  public Command f_idle() {
    return run(io::stop);
  }
}
