package frc.robot.sim;

/**
 * A simulation of a perfect battery that always has a open-circuit voltage of 12 volts, and only loses voltage output
 * from current draw.
 */
public class PerfectBatterySim implements BatterySim {
	private final double nominalVoltage;
	private final double internalResistance;
	private double current = 0;

	public PerfectBatterySim(double nominalVoltage, double internalResistance) {
		this.nominalVoltage = nominalVoltage;
		this.internalResistance = internalResistance;
	}

	/**
	 * Creates a simulation of a nominal 12-volt FRC battery with 15 milliohms of internal resistance.
	 */
	public static PerfectBatterySim nominal() {
		return new PerfectBatterySim(12, 0.015);
	}

	/**
	 * Creates a simulation of an excellent FRC battery with 13 volts nominal and 12 milliohms of internal resistance.
	 */
	public static PerfectBatterySim excellent() {
		return new PerfectBatterySim(13, 0.012);
	}

	/**
	 * Creates a simulation of the ideal 12-volt battery with no internal resistance. This battery will output a constant
	 * 12 volts regardless of load.
	 */
	public static PerfectBatterySim ideal() {
		return new PerfectBatterySim(12, 0);
	}

	@Override
	public void setCurrentDraw(double current) {
		this.current = current;
	}

	@Override
	public void update(double timestep) {
		// Nothing to do here
		// A simulation of a limited-capacity battery like the ones we use on robots
		// would use this
		// function to calculate how much capacity was drawn out from the battery
	}

	@Override
	public double getVoltage() {
		return nominalVoltage - (current * internalResistance);
	}
}
