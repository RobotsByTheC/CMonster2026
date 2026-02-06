package frc.robot.subsystems.shooter.feeder;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.sim.MechanismSim;
import frc.robot.sim.SimulationContext;

import static edu.wpi.first.units.Units.*;

@Logged
public class SimFeederIO implements FeederIO {
  @NotLogged private final FlywheelSim flywheelSim;
  @NotLogged private final MechanismSim flywheelMechanismSim;

  private final MutVoltage targetVoltage = Volts.mutable(0);

  public SimFeederIO() {
    flywheelSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1).withReduction(3), 0.005, 1),
        DCMotor.getNEO(1).withReduction(3));

    flywheelMechanismSim = new MechanismSim() {
      @Override
      public double getCurrentDraw() {
        return flywheelSim.getCurrentDrawAmps();
      }

      @Override
      public void update(double timestep) {
        flywheelSim.setInputVoltage(flywheelMechanismSim.outputVoltage(targetVoltage.in(Volts)));

        flywheelSim.update(timestep);
      }
    };

    SimulationContext.getDefault().addMechanism(flywheelMechanismSim);
  }

  @Override
  public void stop() {
    targetVoltage.mut_setMagnitude(0);
  }

  @Override
  public void setVoltage(Voltage voltage) {
    targetVoltage.mut_setMagnitude(voltage.in(Volts));
  }

  @Override
  public boolean isBallAtFlywheel() {
    return false;
  }

  @Override
  public boolean isBallReadyToFire() {
    return false;
  }

  @Override
  public Current getCurrentDraw() {
    return Amps.of(flywheelSim.getCurrentDrawAmps());
  }
}
