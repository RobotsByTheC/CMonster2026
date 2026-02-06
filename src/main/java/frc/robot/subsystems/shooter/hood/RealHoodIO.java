package frc.robot.subsystems.shooter.hood;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.CANConstants.HOOD_CAN_ID;

@Logged
public class RealHoodIO implements HoodIO {
  private final SparkMax spark;
  private final RelativeEncoder encoder;
  private final SparkLimitSwitch limitSwitch;

  public RealHoodIO() {
    spark = new SparkMax(HOOD_CAN_ID, SparkLowLevel.MotorType.kBrushless);
    encoder = spark.getEncoder();
    limitSwitch = spark.getForwardLimitSwitch();
  }

  @Override
  public void stop() {
    spark.setVoltage(Volts.zero());
  }

  @Override
  public Angle getAngle() {
    return Rotations.of(encoder.getPosition());
  }

  @Override
  public AngularVelocity getVelocity() {
    return RPM.of(encoder.getVelocity());
  }

  @Override
  public void setVoltage(Voltage voltage) {
    spark.setVoltage(voltage);
  }

  @Override
  public boolean atBottom() {
    return limitSwitch.isPressed();
  }
}
