package frc.robot.subsystems.intake;

import static frc.robot.Constants.CANConstants.*;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class RealIntakeIO implements IntakeIO{
  private final SparkMax intakeMotor;
  private final SparkMaxConfig intakeMotorConfig;

  public RealIntakeIO() {
    intakeMotor = new SparkMax(INTAKE_CAN_ID, SparkLowLevel.MotorType.kBrushless);
    intakeMotorConfig = new SparkMaxConfig();
    intakeMotor.configure(intakeMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
  }

  @Override
  public void setIntakeVoltage(Voltage voltage) {
    intakeMotor.setVoltage(voltage);
  }
}