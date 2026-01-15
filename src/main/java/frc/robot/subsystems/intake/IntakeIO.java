package frc.robot.subsystems.intake;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

@Logged
public interface IntakeIO {
  /**
   * Sets the voltage of the intake.
   * Positive voltage is intake, negative is outtake.
   * @param voltage The voltage to set the intake to.
   */
  void setIntakeVoltage(Voltage voltage);

  /**
   * Sets the voltage of the wrist controlling the intake.
   * Positive voltage is clockwise (extending the intake), negative values are counterclockwise (retracting the intake).
   * @param voltage The voltage to set the wrist to.
   */
  void setWristVoltage(Voltage voltage);

  /**
   * Gets the position of the wrist using the absolute encoders on the NEOs.
   * @return The position of the wrist.
   */
  Angle getWristPosition();

  /**
   * Gets the velocity of the wrist using the absolute encoders on the NEOs.
   * @return The velocity of the wrist.
   */
  AngularVelocity getWristVelocity();
}