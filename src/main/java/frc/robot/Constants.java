package frc.robot;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;

public class Constants {
  public static class CANConstants {
    public static final int INTAKE_CAN_ID = 9;
  }
  public static class IntakeConstants {
    public static final Voltage INTAKE_VOLTAGE = Volts.of(5);
    public static final Voltage OUTTAKE_VOLTAGE = Volts.of(-5);
  }
}