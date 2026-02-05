package frc.robot.subsystems.shooter.feeder;

import static edu.wpi.first.units.Units.Amps;

import com.reduxrobotics.sensors.canandcolor.Canandcolor;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

@Logged
public class RealFeederIO implements FeederIO {
	private final SparkMax spark;

	private final Canandcolor bottom;
	private final Canandcolor middle;
	private final Canandcolor top;

	public RealFeederIO(boolean inverted, int sparkCAN, int bottomCAN, int middleCAN, int topCAN) {
		spark = new SparkMax(sparkCAN, SparkLowLevel.MotorType.kBrushless);
		SparkBaseConfig config = new SparkMaxConfig().inverted(inverted).idleMode(SparkBaseConfig.IdleMode.kBrake);
		spark.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		bottom = new Canandcolor(bottomCAN);
		middle = new Canandcolor(middleCAN);
		top = new Canandcolor(topCAN);
	}

	@Override
	public void stop() {
		spark.setVoltage(0);
	}

	@Override
	public Current getCurrentDraw() {
		return Amps.of(spark.getOutputCurrent());
	}

	@Override
	public void setVoltage(Voltage voltage) {
		spark.setVoltage(voltage);
	}

	@Override
	public boolean isBallAtFlywheel() {
		return top.getProximity() < 0.1;
	}

	@Override
	public boolean isBallReadyToFire() {
		return bottom.getProximity() < 0.1;
	}
}
