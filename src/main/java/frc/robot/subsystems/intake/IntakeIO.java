package frc.robot.subsystems.intake;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Voltage;

@Logged
public interface IntakeIO {
  /**
   * Sets the voltage of the intake.
   * Positive voltage is intake, negative is outtake.
   * @param voltage The voltage to set the intake to.
   */
  void setIntakeVoltage(Voltage voltage);
}