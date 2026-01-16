package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.CANConstants.*;
import static frc.robot.Constants.IntakeConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

@Logged
public class RealIntakeIO implements IntakeIO {
  private final SparkMax intakeMotor;
  private final SparkMax wristMotor;
  private final SparkAbsoluteEncoder wristEncoder;

  public RealIntakeIO() {
    intakeMotor = new SparkMax(INTAKE_CAN_ID, SparkLowLevel.MotorType.kBrushless);
    SparkMaxConfig intakeMotorConfig = new SparkMaxConfig();
    intakeMotor.configure(intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    wristMotor = new SparkMax(WRIST_CAN_ID, SparkLowLevel.MotorType.kBrushless);
    SparkMaxConfig wristMotorConfig = new SparkMaxConfig();
    wristMotorConfig.closedLoop.pid(KP, KI, KD).positionWrappingEnabled(true).positionWrappingInputRange(0, 2*Math.PI).feedForward.svag(KS, KV, KA, KG);
    wristMotor.configure(wristMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    wristEncoder = wristMotor.getAbsoluteEncoder();
  }

  @Override
  public void setIntakeVoltage(Voltage voltage) {
    intakeMotor.setVoltage(voltage);
  }

  @Override
  public void setWristPosition(Angle angle) {
    wristMotor.configAccessor.closedLoop;
  }

  @Override
  public Angle getWristPosition() {
    return Rotations.of(wristEncoder.getPosition());
  }

  @Override
  public AngularVelocity getWristVelocity() {
    return RPM.of(wristEncoder.getVelocity());
  }

  @Override
  public Voltage getWristVoltage() {
    return Volts.of(wristMotor.getAppliedOutput() * wristMotor.getBusVoltage());
  }
}