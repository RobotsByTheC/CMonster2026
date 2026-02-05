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
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants.FlywheelConstants;

@Logged
public class RealFeederIO implements FeederIO {
	private final SparkMax spark;

  private final Canandcolor bottom;
  private final Canandcolor middle;
  private final Canandcolor top;

  public RealFeederIO(boolean inverted, int sparkCAN) {
		spark = new SparkMax(sparkCAN, SparkLowLevel.MotorType.kBrushless);
		SparkBaseConfig config = new SparkMaxConfig().inverted(inverted).idleMode(SparkBaseConfig.IdleMode.kBrake);
		spark.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    bottom = new Canandcolor(Constants.CANConstants.LEFT_CNC_BOTTOM);
    middle = new Canandcolor(Constants.CANConstants.LEFT_CNC_MIDDLE);
    top = new Canandcolor(Constants.CANConstants.LEFT_CNC_TOP);
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
    return top.getProximity()<0.1;
  }

  @Override
  public boolean isBallReadyToFire() {
    return bottom.getProximity()<0.1;
  }
}
