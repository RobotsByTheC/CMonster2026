package frc.robot.subsystems.feeder;

import static edu.wpi.first.units.Units.Amps;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.ShooterConstants.FlywheelConstants;

@Logged
public class RealFeederIO implements FeederIO {
	private final SparkMax spark;

	public RealFeederIO(boolean inverted, int can) {
		spark = new SparkMax(can, SparkLowLevel.MotorType.kBrushless);
		SparkBaseConfig config = new SparkMaxConfig().inverted(inverted).idleMode(SparkBaseConfig.IdleMode.kCoast);
		config.closedLoop.pid(FlywheelConstants.KP, FlywheelConstants.KI, FlywheelConstants.KD);
		spark.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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

}
