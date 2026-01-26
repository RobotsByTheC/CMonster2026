package frc.robot.subsystems.shooter;

import edu.wpi.first.math.Pair;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

import java.util.HashMap;
import java.util.Map;

import static edu.wpi.first.units.Units.*;

public record LookupTable() {
  public static final Map<Integer, Pair<AngularVelocity, Angle>> SHOOTER_LOOKUP_TABLE = new HashMap<>();
  static {
    SHOOTER_LOOKUP_TABLE.put(1,  new Pair<>(RPM.of(1000), Degrees.of(80)));
    SHOOTER_LOOKUP_TABLE.put(2,  new Pair<>(RPM.of(1100), Degrees.of(75)));
    SHOOTER_LOOKUP_TABLE.put(3,  new Pair<>(RPM.of(1200), Degrees.of(70)));
    SHOOTER_LOOKUP_TABLE.put(4,  new Pair<>(RPM.of(1300), Degrees.of(65)));
    SHOOTER_LOOKUP_TABLE.put(5,  new Pair<>(RPM.of(1400), Degrees.of(60)));
    SHOOTER_LOOKUP_TABLE.put(6,  new Pair<>(RPM.of(1500), Degrees.of(55)));
    SHOOTER_LOOKUP_TABLE.put(7,  new Pair<>(RPM.of(1600), Degrees.of(50)));
    SHOOTER_LOOKUP_TABLE.put(8,  new Pair<>(RPM.of(1700), Degrees.of(45)));
    SHOOTER_LOOKUP_TABLE.put(9,  new Pair<>(RPM.of(1800), Degrees.of(40)));
    SHOOTER_LOOKUP_TABLE.put(10,  new Pair<>(RPM.of(1900), Degrees.of(35)));
  }
}