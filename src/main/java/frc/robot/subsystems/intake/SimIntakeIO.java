package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.sim.MechanismSim;
import frc.robot.sim.SimulationContext;

@Logged
public class SimIntakeIO implements IntakeIO {
  @NotLogged private final SingleJointedArmSim wristSim;
  @NotLogged private final DCMotorSim motorSim;
  @NotLogged private final MechanismSim mechanismSim;

  public SimIntakeIO() {
    wristSim = new SingleJointedArmSim(DCMotor.getNEO(1), 60, 0.1, 0.3, 0, Math.PI, true, 0);
    motorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.1, 20), DCMotor.getNEO(1));
    mechanismSim = new MechanismSim() {
      @Override
      public double getCurrentDraw() {
        return motorSim.getCurrentDrawAmps();
      }

      @Override
      public void update(double timestep) {
        motorSim.update(timestep);
        wristSim.update(timestep);
      }
    };
    SimulationContext.getDefault().addMechanism(mechanismSim);
  }

  @Override
  public void setIntakeVoltage(Voltage voltage) {
    motorSim.setInputVoltage(mechanismSim.outputVoltage(voltage.in(Volts)));
  }

  @Override
  public void setWristVoltage(Voltage voltage) {

  }

  @Override
  public Angle getWristPosition() {
    return null;
  }

  @Override
  public AngularVelocity getWristVelocity() {
    return null;
  }

  public AngularVelocity getMotorVelocity() {
    return RPM.of(motorSim.getAngularVelocityRPM());
  }
}