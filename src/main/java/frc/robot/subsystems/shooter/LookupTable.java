package frc.robot.subsystems.shooter;

import edu.wpi.first.math.Pair;
import frc.robot.subsystems.data.InterpolatingShooterTreeMap;

import static edu.wpi.first.units.Units.*;

public record LookupTable() {
	public static final InterpolatingShooterTreeMap SHOOTER_LOOKUP_TABLE = new InterpolatingShooterTreeMap();
	static {
		SHOOTER_LOOKUP_TABLE.put(Meters.of(0.1), new Pair<>(RPM.of(1000), Degrees.of(80)));
		SHOOTER_LOOKUP_TABLE.put(Meters.of(0.2), new Pair<>(RPM.of(1100), Degrees.of(75)));
		SHOOTER_LOOKUP_TABLE.put(Meters.of(0.3), new Pair<>(RPM.of(1200), Degrees.of(70)));
		SHOOTER_LOOKUP_TABLE.put(Meters.of(0.4), new Pair<>(RPM.of(1300), Degrees.of(65)));
		SHOOTER_LOOKUP_TABLE.put(Meters.of(0.5), new Pair<>(RPM.of(1400), Degrees.of(60)));
		SHOOTER_LOOKUP_TABLE.put(Meters.of(0.6), new Pair<>(RPM.of(1500), Degrees.of(55)));
		SHOOTER_LOOKUP_TABLE.put(Meters.of(0.7), new Pair<>(RPM.of(1600), Degrees.of(50)));
		SHOOTER_LOOKUP_TABLE.put(Meters.of(0.8), new Pair<>(RPM.of(1700), Degrees.of(45)));
		SHOOTER_LOOKUP_TABLE.put(Meters.of(0.9), new Pair<>(RPM.of(1800), Degrees.of(40)));
		SHOOTER_LOOKUP_TABLE.put(Meters.of(1), new Pair<>(RPM.of(1900), Degrees.of(35)));
	}
}
