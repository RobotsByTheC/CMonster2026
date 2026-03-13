package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.IntakeConstants.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;

@Logged
public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final Roller roller;
  private final Extension extension;
  private final ArmFeedforward feedforward;
  private final ProfiledPIDController pidController;

  class Extension extends SubsystemBase {
    public Command applyVoltage(Supplier<Voltage> voltage) {
      return run(() -> io.setWristVoltage(voltage.get()));
    }

    public Command stop() {
      return runOnce(() -> io.setWristVoltage(Volts.zero()));
    }
  }

  class Roller extends SubsystemBase {
    public Command runIntakeMotor() {
      return runOnce(() -> io.setIntakeVoltage(INTAKE_VOLTAGE));
    }

    public Command stop() {
      return runOnce(() -> io.setIntakeVoltage(Volts.zero()));
    }

    public Command reverseIntakeMotor() {
      return runOnce(() -> io.setIntakeVoltage(OUTTAKE_VOLTAGE));
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
    return roller.runIntakeMotor().alongWith(Commands.run(() -> System.out.println("blegg")));
  }

  public Command applyVoltageToPivot(Supplier<Voltage> voltage) {
    Command command = extension.applyVoltage(voltage)
        .alongWith(Commands.run(() -> System.out.println("blegg " + voltage.get())));
    return command;
  }

  public Command zero() {
    return runOnce(() -> io.zero());
  }
}
