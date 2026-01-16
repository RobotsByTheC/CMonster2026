package frc.robot.subsystems.intake;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.sim.MechanismSim;
import frc.robot.sim.SimulationContext;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.IntakeConstants.KD;
import static frc.robot.Constants.IntakeConstants.KI;
import static frc.robot.Constants.IntakeConstants.KP;
import static frc.robot.Constants.IntakeConstants.MAX_WRIST_ACCELERATION;
import static frc.robot.Constants.IntakeConstants.MAX_WRIST_SPEED;
import static frc.robot.Constants.IntakeConstants.WRIST_TOLERANCE;

@Logged
public class SimIntakeIO implements IntakeIO {
  @NotLogged private final SingleJointedArmSim wristSim;
  @NotLogged private final MechanismSim wristMechanismSim;
  @NotLogged private final DCMotorSim motorSim;
  @NotLogged private final MechanismSim motorMechanismSim;

  private final ProfiledPIDController pidController;

  public SimIntakeIO() {
    wristSim = new SingleJointedArmSim(DCMotor.getNEO(1), 60, 0.1, 0.3, 0, Math.PI, true, 0);
    motorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.1, 20), DCMotor.getNEO(1));
    motorMechanismSim = new MechanismSim() {
      @Override
      public double getCurrentDraw() {
        return motorSim.getCurrentDrawAmps();
      }

      @Override
      public void update(double timestep) {
        motorSim.update(timestep);
      }
    };
    wristMechanismSim = new MechanismSim() {
      @Override
      public double getCurrentDraw() {
        return wristSim.getCurrentDrawAmps();
      }

      @Override
      public void update(double timestep) {
        wristSim.update(timestep);
      }
    };

    pidController = new ProfiledPIDController(KP, KI, KD, new TrapezoidProfile.Constraints(MAX_WRIST_SPEED.in(RadiansPerSecond), MAX_WRIST_ACCELERATION.in(RadiansPerSecondPerSecond)));
    pidController.setTolerance(WRIST_TOLERANCE.in(Radians));
    pidController.enableContinuousInput(0, 2*Math.PI);

    SimulationContext.getDefault().addMechanism(motorMechanismSim);
    SimulationContext.getDefault().addMechanism(wristMechanismSim);
  }

  @Override
  public void setIntakeVoltage(Voltage voltage) {
    motorSim.setInputVoltage(motorMechanismSim.outputVoltage(voltage.in(Volts)));
  }

  @Override
  public void setWristPosition(Angle angle) {
    wristSim.setInputVoltage(wristMechanismSim.outputVoltage(pidController.calculate(wristSim.getAngleRads(), angle.in(Radians))));
  }

  @Override
  public Angle getWristPosition() {
    return Radians.of(wristSim.getAngleRads());
  }

  @Override
  public AngularVelocity getWristVelocity() {
    return RadiansPerSecond.of(wristSim.getVelocityRadPerSec());
  }

  @Override
  public Voltage getWristVoltage() {
    return Volts.of(wristSim.getInput(0));
  }

  public AngularVelocity getMotorVelocity() {
    return RPM.of(motorSim.getAngularVelocityRPM());
  }
}