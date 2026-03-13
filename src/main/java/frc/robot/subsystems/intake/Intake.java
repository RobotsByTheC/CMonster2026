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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final Roller roller;
  private final Extension extension;
  private final ArmFeedforward feedforward;
  private final ProfiledPIDController pidController;

  class Extension extends SubsystemBase {
    public Command extend() {
      return rotateToAngle(WRIST_EXTEND_ANGLE);
    }

    public Command stow() {
      return rotateToAngle(WRIST_STOW_ANGLE);
    }

    public Command stop() {
      return runOnce(() -> io.setWristVoltage(Volts.zero()));
    }

    private Command rotateToAngle(Angle targetAngle) {
      return startRun(
          () -> pidController.reset(io.getWristPosition().in(Radians), io.getWristVelocity().in(RadiansPerSecond)),
          () -> io.setWristVoltage(calculatePIDVoltage(targetAngle)));
    }

    private Voltage calculatePIDVoltage(Angle targetAngle) {
      return Volts.of(pidController.calculate(io.getWristPosition().in(Radians), targetAngle.in(Radians))
          + feedforward.calculate(io.getWristPosition().in(Radians), io.getWristVelocity().in(RadiansPerSecond)));
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

  public Command f_idle() {
    Command command = extension.stop().alongWith(roller.stop());
    command.addRequirements(this);
    return command;
  }

  public Command f_extend() {
    return extension.extend();
  }

  public Command l_retractAndGrab() {
    return extension.stow().until(pidController::atSetpoint);
  }

  public Command f_activate_rollers() {
    return roller.runIntakeMotor();
  }
}
