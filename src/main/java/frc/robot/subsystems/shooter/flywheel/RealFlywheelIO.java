package frc.robot.subsystems.shooter.flywheel;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
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
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.revroboticshacks.HackedSparkMaxAlternateEncoder;

@Logged
public class RealFlywheelIO implements FlywheelIO {
  private final SparkMax leadMotor;
  private final SparkMax followerMotor;

  private final SparkClosedLoopController controller;
  private final RelativeEncoder primaryEncoder;
  private final RelativeEncoder followerEncoder;
  private final MutAngularVelocity target = RPM.mutable(0);

  public RealFlywheelIO(boolean inverted, int leaderId, int followerId, double P, double I, double D, double S,
      double V) {
    leadMotor = new SparkMax(leaderId, SparkLowLevel.MotorType.kBrushless);
    SparkMaxConfig leadConfig = (SparkMaxConfig) new SparkMaxConfig().inverted(inverted)
        .idleMode(SparkBaseConfig.IdleMode.kCoast);
    leadConfig.closedLoop.pid(P, I, D);
    leadConfig.closedLoop.feedForward.sv(S, V);
    leadConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    leadConfig.smartCurrentLimit(40);
    leadConfig.encoder.quadratureAverageDepth(2).quadratureMeasurementPeriod(1);

    leadMotor.configure(leadConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    followerMotor = new SparkMax(followerId, SparkLowLevel.MotorType.kBrushless);
    SparkMaxConfig followerConfig = (SparkMaxConfig) new SparkMaxConfig().follow(leadMotor, true)
        .idleMode(SparkBaseConfig.IdleMode.kCoast);
    followerConfig.smartCurrentLimit(40);

    followerMotor.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    controller = leadMotor.getClosedLoopController();
    primaryEncoder = leadMotor.getEncoder();
    followerEncoder = followerMotor.getEncoder();
  }

  @Override
  public AngularVelocity getAlternateVelocity() {
    return RPM.of(0);
  }

  @Override
  public AngularVelocity getPrimaryVelocity() {
    return RPM.of(primaryEncoder.getVelocity());
  }
  @Override
  public AngularVelocity getFollowerVelocity() {
    return RPM.of(followerEncoder.getVelocity());
  }

  @Override
  public AngularVelocity getTargetVelocity() {
    return target;
  }

  @Override
  public Current getCurrentDraw() {
    return Amps.of(leadMotor.getOutputCurrent() + followerMotor.getOutputCurrent());
  }

  @Override
  public Voltage getVoltageDraw() {
    double leadVolts = leadMotor.getAppliedOutput() * leadMotor.getBusVoltage();
    double followerVolts = followerMotor.getAppliedOutput() * followerMotor.getBusVoltage();
    return Volts.of((leadVolts + followerVolts) / 2);
  }

  @Override
  public void setVelocity(AngularVelocity velocity) {
    controller.setSetpoint(velocity.in(RPM), SparkBase.ControlType.kVelocity);
    target.mut_setMagnitude(velocity.in(RPM));
  }

  @Override
  public void setVoltage(Voltage voltage) {
    leadMotor.setVoltage(voltage);
  }

  @Override
  public Angle getAlternatePosition() {
    return Radians.zero();
  }
  @Override
  public Angle getPrimaryPosition() {
    return Rotations.of(primaryEncoder.getPosition());
  }
  @Override
  public Angle getFollowerPosition() {
    return Rotations.of(followerEncoder.getPosition());
  }

  @Override
  public boolean atTargetVelocity() {
    return RPM.of(controller.getSetpoint()).isNear(getAlternateVelocity(), RPM.of(20));
  }

  @Override
  public Current getCurrentA() {
    return Amps.of(leadMotor.getOutputCurrent());
  }

  @Override
  public Current getCurrentB() {
    return Amps.of(followerMotor.getOutputCurrent());
  }

  @Override
  public Voltage getVoltageA() {
    return Volts.of(leadMotor.getAppliedOutput() * leadMotor.getBusVoltage());
  }

  @Override
  public Voltage getVoltageB() {
    return Volts.of(followerMotor.getAppliedOutput() * followerMotor.getBusVoltage());
  }
}
