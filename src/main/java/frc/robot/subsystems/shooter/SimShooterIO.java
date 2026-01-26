package frc.robot.subsystems.shooter;

import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.epilogue.Logged;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.ShooterConstants.FlywheelConstants;
import static frc.robot.Constants.ShooterConstants.HoodConstants;

import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.sim.MechanismSim;
import frc.robot.sim.SimulationContext;

@Logged
public class SimShooterIO implements ShooterIO {
	@NotLogged private final FlywheelSim flywheelSim;
	@NotLogged private final MechanismSim flywheelMechanismSim;

	@NotLogged private final SingleJointedArmSim hoodSim;
	@NotLogged private final MechanismSim hoodMechanismSim;

	private final SimpleMotorFeedforward flywheelFeedForward;
	private final PIDController flywheelPIDController;

	private final PIDController hoodPIDController;

	private final MutAngularVelocity targetVelocity = RadiansPerSecond.mutable(0);

	public SimShooterIO() {
		flywheelSim = new FlywheelSim(
				LinearSystemId.createFlywheelSystem(DCMotor.getNEO(4).withReduction(1 / 1.5), 6 / 3417.2, 1),
				DCMotor.getNEO(4).withReduction(1 / 1.5));
		hoodSim = new SingleJointedArmSim(DCMotor.getNEO(1), 60, 0.1, 0.3, 0, Math.PI, true, 0);

		flywheelMechanismSim = new MechanismSim() {
			@Override
			public double getCurrentDraw() {
				return flywheelSim.getCurrentDrawAmps();
			}

			@Override
			public void update(double timestep) {
				flywheelSim.update(timestep);
			}
		};
		hoodMechanismSim = new MechanismSim() {
			@Override
			public double getCurrentDraw() {
				return hoodSim.getCurrentDrawAmps();
			}

			@Override
			public void update(double timestep) {
				hoodSim.update(timestep);
			}
		};

		flywheelFeedForward = new SimpleMotorFeedforward(FlywheelConstants.KS, FlywheelConstants.KV);
		flywheelPIDController = new PIDController(FlywheelConstants.KP, FlywheelConstants.KI, FlywheelConstants.KD);
		hoodPIDController = new PIDController(HoodConstants.KP, HoodConstants.KI, HoodConstants.KD);

		SimulationContext.getDefault().addMechanism(flywheelMechanismSim);
		SimulationContext.getDefault().addMechanism(hoodMechanismSim);
	}

	@Override
	public void stopFlywheel() {
		flywheelSim.setInputVoltage(0);
	}

	@Override
	public void stopHood() {
		hoodSim.setInputVoltage(0);
	}

	@Override
	public void setFlywheelVelocity(AngularVelocity angularVelocity) {
		double feedForward = flywheelFeedForward.calculate(angularVelocity.in(RPM));
		double pid = flywheelPIDController.calculate(flywheelSim.getAngularVelocity().in(RPM), angularVelocity.in(RPM));

		flywheelSim.setInputVoltage(flywheelMechanismSim.outputVoltage(pid + feedForward));
	}

	@Override
	public void setHoodAngle(Angle angle) {
		hoodSim.setInputVoltage(
				hoodMechanismSim.outputVoltage(hoodPIDController.calculate(hoodSim.getAngleRads(), angle.in(Radians))));
	}

	@Override
	public AngularVelocity getFlywheelVelocity() {
		return flywheelSim.getAngularVelocity();
	}

	@Override
	public Angle getHoodAngle() {
		return Radians.of(hoodSim.getAngleRads());
	}

	@Override
	public Current getCurrentDraw() {
		return Amps.of(flywheelSim.getCurrentDrawAmps());
	}
}
