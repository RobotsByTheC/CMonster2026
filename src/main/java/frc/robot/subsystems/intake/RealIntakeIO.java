package frc.robot.subsystems.intake;

import static frc.robot.Constants.CANConstants.*;


import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Voltage;

@Logged
public class RealIntakeIO implements IntakeIO{
  private final SparkMax intakeMotor;
  private final SparkMaxConfig intakeMotorConfig;

  public RealIntakeIO() {
    intakeMotor = new SparkMax(INTAKE_CAN_ID, SparkLowLevel.MotorType.kBrushless);
    intakeMotorConfig = new SparkMaxConfig();
    intakeMotor.configure(intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void setIntakeVoltage(Voltage voltage) {
    intakeMotor.setVoltage(voltage);
  }
}