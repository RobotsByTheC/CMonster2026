package frc.robot.subsystems.shooter.flywheel;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.CANConstants.*;
import static frc.robot.Constants.ShooterConstants.FlywheelConstants;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MutAngularVelocity;

@Logged
public class RealFlywheelIO implements FlywheelIO {
	private final SparkMax sparkA;
	private final SparkMax sparkB;

	private final SparkClosedLoopController controller;
	private final RelativeEncoder encoder;
  private final MutAngularVelocity target = RPM.mutable(0);

	public RealFlywheelIO(boolean inverted, int canA, int canB) {
		sparkA = new SparkMax(canA, SparkLowLevel.MotorType.kBrushless);
		SparkBaseConfig configA = new SparkMaxConfig().inverted(inverted).idleMode(SparkBaseConfig.IdleMode.kCoast);
		configA.closedLoop.pid(FlywheelConstants.KP, FlywheelConstants.KI, FlywheelConstants.KD);
		sparkA.configure(configA, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		sparkB = new SparkMax(canB, SparkLowLevel.MotorType.kBrushless);
    SparkBaseConfig configB = new SparkMaxConfig().follow(sparkA, true).idleMode(SparkBaseConfig.IdleMode.kCoast);
    configB.closedLoop.pid(FlywheelConstants.KP, FlywheelConstants.KI, FlywheelConstants.KD);
    configA.encoder.velocityConversionFactor(1);
    configA.encoder.positionConversionFactor(1);

    sparkB.configure(configB, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
		controller = sparkA.getClosedLoopController();
		encoder = sparkA.getEncoder();
	}

	@Override
	public void stop() {
		controller.setSetpoint(0, SparkBase.ControlType.kVoltage);
	}

	@Override
	public AngularVelocity getVelocity() {
		return RPM.of(encoder.getVelocity());
	}

	@Override
	public Current getCurrentDraw() {
		return Amps.of(sparkA.getOutputCurrent() + sparkB.getOutputCurrent());
	}

	@Override
	public void setVelocity(AngularVelocity velocity) {
		controller.setSetpoint(velocity.in(RPM), SparkBase.ControlType.kVelocity);
    target.mut_setMagnitude(velocity.in(RPM));
    System.out.println("t: " + velocity + ", a: " + controller.getSetpoint());
	}
}
