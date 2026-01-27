package frc.robot.subsystems.shooter;

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
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

import java.util.function.Supplier;

@Logged
public class RealShooterIO implements ShooterIO {
	private final SparkMax leftShooterSparkA;
	private final SparkMax leftShooterSparkB;
	private final SparkMax rightShooterSparkA;
	private final SparkMax rightShooterSparkB;
	private final SparkMax hoodSpark;

	private final SparkClosedLoopController leftShooterController;
	private final SparkClosedLoopController rightShooterController;
	private final SparkClosedLoopController hoodController;

	private final RelativeEncoder leftShooterEncoder;
	private final RelativeEncoder rightShooterEncoder;
	private final RelativeEncoder hoodEncoder;

	public RealShooterIO() {
		leftShooterSparkA = new SparkMax(FLYWHEEL_LEFT_A_CAN_ID, SparkLowLevel.MotorType.kBrushless);
		SparkBaseConfig leftConfigA = new SparkMaxConfig();
		leftConfigA.closedLoop.pid(FlywheelConstants.KP, FlywheelConstants.KI, FlywheelConstants.KD);
		leftShooterSparkA.configure(leftConfigA, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
		leftShooterSparkB = new SparkMax(FLYWHEEL_LEFT_B_CAN_ID, SparkLowLevel.MotorType.kBrushless);
		SparkBaseConfig leftConfigB = new SparkMaxConfig().follow(leftShooterSparkA, true);
		leftShooterSparkB.configure(leftConfigB, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
		rightShooterSparkA = new SparkMax(FLYWHEEL_RIGHT_A_CAN_ID, SparkLowLevel.MotorType.kBrushless);
		SparkMaxConfig rightConfigA = new SparkMaxConfig();
		rightConfigA.closedLoop.pid(FlywheelConstants.KP, FlywheelConstants.KI, FlywheelConstants.KD);
		rightShooterSparkA.configure(rightConfigA, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
		rightShooterSparkB = new SparkMax(FLYWHEEL_RIGHT_B_CAN_ID, SparkLowLevel.MotorType.kBrushless);
		SparkBaseConfig rightConfigB = new SparkMaxConfig().follow(rightShooterSparkA, true);
		rightShooterSparkB.configure(rightConfigB, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		leftShooterController = leftShooterSparkA.getClosedLoopController();
		rightShooterController = rightShooterSparkB.getClosedLoopController();
		leftShooterEncoder = leftShooterSparkA.getEncoder();
		rightShooterEncoder = rightShooterSparkA.getEncoder();

		hoodSpark = new SparkMax(HOOD_CAN_ID, SparkLowLevel.MotorType.kBrushless);
		hoodSpark.configure(new SparkMaxConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
		hoodController = hoodSpark.getClosedLoopController();
		hoodEncoder = hoodSpark.getEncoder();
	}

	@Override
	public void stopFlywheel() {
		leftShooterSparkA.setVoltage(0);
		leftShooterSparkB.setVoltage(0);
		rightShooterSparkA.setVoltage(0);
		rightShooterSparkB.setVoltage(0);
		leftShooterController.setSetpoint(0, SparkBase.ControlType.kVoltage);
		rightShooterController.setSetpoint(0, SparkBase.ControlType.kVoltage);
	}

	@Override
	public void stopHood() {
		hoodSpark.setVoltage(0);
		hoodController.setSetpoint(0, SparkBase.ControlType.kVoltage);
	}

	@Override
	public AngularVelocity getFlywheelVelocity() {
		return RPM.of((leftShooterEncoder.getVelocity() + rightShooterEncoder.getVelocity()) / 2);
	}

	@Override
	public Angle getHoodAngle() {
		return Radians.of(hoodEncoder.getPosition());
	}

	@Override
	public Current getCurrentDraw() {
		return null;
	}

	@Override
	public void setFlywheelVelocity(AngularVelocity velocity) {
		leftShooterController.setSetpoint(velocity.in(RadiansPerSecond), SparkBase.ControlType.kVelocity);
		rightShooterController.setSetpoint(velocity.in(RadiansPerSecond), SparkBase.ControlType.kVelocity);
	}

	@Override
	public void setHoodAngle(Angle angle) {
		hoodController.setSetpoint(angle.in(Radians), SparkBase.ControlType.kPosition);
	}
}
