package frc.robot.sim;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import java.util.ArrayList;
import java.util.Collection;

/**
 * Responsible for coordinating periodic simulation updates. Provide a {@link BatterySim battery}
 * and a set of {@link MechanismSim mechanisms} to simulate, then call {@link #update(double)} to
 * step the simulation forward. It is recommended to call {@code update} in a robot's {@link
 * TimedRobot#simulationPeriodic()} method.
 */
public class SimulationContext {
  private final BatterySim battery;
  private final Collection<MechanismSim> mechanisms;
  private final Collection<Simulation> simulations;

  private static final SimulationContext defaultSimulation =
      new SimulationContext(PerfectBatterySim.nominal());

  /**
   * Gets the default simulation context. Useful for convenient simulation registration, but should
   * not be used in unit tests. The simulation uses a nominal 12-volt battery with no capacity loss
   * over time, i.e. it will always output exactly 12 volts when not under load.
   */
  public static SimulationContext getDefault() {
    return defaultSimulation;
  }

  public BatterySim getBattery() {
    return battery;
  }

  /**
   * Creates a new simulation context.
   *
   * @param battery the battery the simulation should use.
   */
  public SimulationContext(BatterySim battery) {
    this.battery = battery;
    this.mechanisms = new ArrayList<>();
    this.simulations = new ArrayList<>();
  }

  /**
   * Adds a mechanism to the simulation.
   *
   * @param mechanism the mechanism to add
   * @param <M> the type of the mechanism simulation
   * @return the added mechanism, or null if the mechanism could not be added
   */
  public <M extends MechanismSim> M addMechanism(M mechanism) {
    if (!mechanisms.contains(mechanism) && mechanisms.add(mechanism)) {
      return mechanism;
    } else {
      return null;
    }
  }

  /**
   * Removes a mechanism from simulation. Does nothing if the mechanism is not already in the
   * simulation.
   *
   * @param mechanism the mechanism to remove
   */
  public void removeMechanism(MechanismSim mechanism) {
    mechanisms.remove(mechanism);
  }

  /**
   * Adds a generic simulation to run periodically. Mechanism simulations should be added with
   * {@link #addMechanism(MechanismSim)} instead of with this method
   *
   * @param sim the simulation to add
   */
  public void addPeriodic(Simulation sim) {
    if (!simulations.contains(sim)) {
      simulations.add(sim);
    }
  }

  /**
   * Removes a generic simulation to no longer run it as part of the periodic update loop.
   *
   * @param sim the simulation to remove
   */
  public void removePeriodic(Simulation sim) {
    simulations.remove(sim);
  }

  public void removeAll() {
    simulations.clear();
    mechanisms.clear();
  }

  /**
   * Updates the simulation by moving it forward one timestep. This will update all the mechanism
   * simulations currently in this context and update the robot's battery voltage to account for the
   * voltage drop caused by the currents drawn by all the mechanisms. The voltage can be read via
   * {@link RobotController#getBatteryVoltage()} or from a mechanism simulation's {@link
   * MechanismSim#getBatteryVoltage()} convenience method.
   *
   * @param timestep how far forward to move the simulation, in seconds.
   */
  public void update(double timestep) {
    double totalCurrentDraw = 0;

    for (var mechanism : mechanisms) {
      mechanism.update(timestep);
      totalCurrentDraw += mechanism.getCurrentDraw();
    }

    // Update generic simulations after mechanisms; subsystem simulations, in particular, should
    // run after all their mechanisms to give them same-tick data
    for (var simulation : simulations) {
      simulation.update(timestep);
    }

    // Calculate the new battery voltage. Mechanism simulations can read it via getBatteryVoltage()
    battery.setCurrentDraw(totalCurrentDraw);
    battery.update(timestep);
    RoboRioSim.setVInVoltage(battery.getVoltage());
    RoboRioSim.setVInCurrent(totalCurrentDraw);
  }
}