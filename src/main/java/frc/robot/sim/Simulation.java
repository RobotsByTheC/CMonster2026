package frc.robot.sim;

@FunctionalInterface
public interface Simulation {
	/**
	 * Updates the simulation by stepping the simulation forward in time.
	 *
	 * @param timestep
	 *            how much time has elapsed since the most recent update, in
	 *            seconds.
	 */
	void update(double timestep);
}
