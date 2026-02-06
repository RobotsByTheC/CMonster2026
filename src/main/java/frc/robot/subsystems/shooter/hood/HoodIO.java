package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

@Logged
public interface HoodIO {
  void stop();
  Angle getAngle();
  AngularVelocity getVelocity();
  void setVoltage(Voltage voltage);
  boolean atBottom();
}
