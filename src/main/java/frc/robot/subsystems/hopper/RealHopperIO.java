package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.Amps;
import static frc.robot.Constants.CANConstants.*;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

@Logged
public class RealHopperIO implements HopperIO {
	private final SparkMax spark;

	public RealHopperIO() {
		spark = new SparkMax(HOPPER_CAN_ID, SparkLowLevel.MotorType.kBrushless);
		SparkBaseConfig config = new SparkMaxConfig().idleMode(SparkBaseConfig.IdleMode.kBrake);
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
