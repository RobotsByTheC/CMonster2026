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
  private final Trigger isPressed;

  public RealHoodIO() {
    spark = new SparkMax(HOOD_CAN_ID, SparkLowLevel.MotorType.kBrushless);
    SparkBaseConfig config = new SparkMaxConfig().inverted(false);
    config.encoder.positionConversionFactor(1d/157.5);
    config.encoder.velocityConversionFactor(1d/157.5);
    config.limitSwitch.forwardLimitSwitchType(LimitSwitchConfig.Type.kNormallyClosed);
    spark.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    encoder = spark.getEncoder();
    limitSwitch = spark.getForwardLimitSwitch();

    isPressed = new Trigger(limitSwitch::isPressed);

    isPressed.onTrue(Commands.runOnce(() -> encoder.setPosition(0)));
  }

  @Override
  public void stop() {
    spark.setVoltage(Volts.zero());
  }

  @Override
  public Angle getAngle() {
    return Rotations.of(encoder.getPosition()).unaryMinus();
  }

  @Override
  public AngularVelocity getVelocity() {
    return RPM.of(encoder.getVelocity());
  }

  @Override
  public Voltage getVoltage() {
    return Volts.of(spark.getAppliedOutput()*spark.getBusVoltage());
  }
  @Override
  public Current getCurrent() {
    return Amps.of(spark.getOutputCurrent());
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
