package frc.robot;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.*;

public class Constants {
  public static class InputConstants {
    public static final int CONTROLLER_PORT = 1;
  }

  public static class CANConstants {
    public static final int INTAKE_CAN_ID = 9;
    public static final int WRIST_CAN_ID = 10;
  }
  public static class IntakeConstants {
    public static final double KP = 5;
    public static final double KI = 0;
    public static final double KD = 0.85;
    public static final double KS = 0.014847;
    public static final double KG = 0.3732;
    public static final double KV = 1.1878;
    public static final double KA = 0.0024773;

    public static final Voltage INTAKE_VOLTAGE = Volts.of(5);
    public static final Voltage OUTTAKE_VOLTAGE = Volts.of(-5);
    public static final AngularVelocity MAX_WRIST_SPEED = RadiansPerSecond.of(10);
    public static final AngularAcceleration MAX_WRIST_ACCELERATION = RadiansPerSecondPerSecond.of(3);
    public static final Angle WRIST_STOW_ANGLE = Radians.of(Math.PI);
    public static final Angle WRIST_EXTEND_ANGLE = Degrees.of(30);
  }
}