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
   * Moves the wrist to the target angle by calculating the voltage that should be applied via PID.
   * @param angle The angle that the wrist should go to.
   */
  void setWristPosition(Angle angle);

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

  /**
   * Gets the voltage being applied to the wrist by the subsystem.
   * @return The voltage being applied to the wrist.
   */
  Voltage getWristVoltage();
}