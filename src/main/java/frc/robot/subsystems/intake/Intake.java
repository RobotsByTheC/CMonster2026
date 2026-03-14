package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.IntakeConstants.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.ConstantTuner;
import java.util.function.Supplier;

@Logged
public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final Roller roller;
  private final Extension extension;
  private final ArmFeedforward feedforward;
  private final ProfiledPIDController pidController;

  class Extension extends SubsystemBase {
    public Command applyVoltage(Voltage voltage) {
      return run(() -> io.setWristVoltage(voltage));
    }

    public Command stop() {
      return runOnce(() -> io.setWristVoltage(Volts.zero()));
    }
  }

  class Roller extends SubsystemBase {
    public Command runIntakeMotor() {
      return run(() -> io.setIntakeVoltage(INTAKE_VOLTAGE));
    }

    public Command stop() {
      return run(() -> io.setIntakeVoltage(Volts.zero()));
    }

    public Command reverseIntakeMotor() {
      return run(() -> io.setIntakeVoltage(OUTTAKE_VOLTAGE));
    }
  }

  public Intake(IntakeIO io) {
    this.io = io;
    roller = new Roller();
    roller.setDefaultCommand(roller.stop());
    extension = new Extension();
    extension.setDefaultCommand(extension.stop());
    pidController = new ProfiledPIDController(KP, KI, KD, new TrapezoidProfile.Constraints(
        MAX_WRIST_SPEED.in(RadiansPerSecond), MAX_WRIST_ACCELERATION.in(RadiansPerSecondPerSecond)));
    pidController.setTolerance(0.0001);
    pidController.enableContinuousInput(0, 2 * Math.PI);
    feedforward = new ArmFeedforward(KS, KG, KV, KA);
  }

  public Command applyVoltageToRollers() {
    return roller.runIntakeMotor();
  }

  public Command f_pivotUp() {
    return extension.applyVoltage(UP_VOLTAGE);
  }

  public Command f_pivotDown() {
    return extension.applyVoltage(DOWN_VOLTAGE);
  }

  public Command zero() {
    return runOnce(io::zero);
  }

  public Command reverseIntakeMotor() {
    return roller.reverseIntakeMotor();
  }
}
