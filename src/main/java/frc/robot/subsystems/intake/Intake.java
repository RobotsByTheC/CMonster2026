package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.IntakeConstants.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class Intake extends SubsystemBase {
  private final IntakeIO io;

  public Intake(IntakeIO io) {
    this.io = io;
  }

  public Command intake() {
    return run(() -> io.setIntakeVoltage(INTAKE_VOLTAGE));
  }

  public Command stop() {
    return run(() -> io.setIntakeVoltage(Volts.zero()));
  }

  public Command outtake() {
    return run(() -> io.setIntakeVoltage(OUTTAKE_VOLTAGE));
  }
}