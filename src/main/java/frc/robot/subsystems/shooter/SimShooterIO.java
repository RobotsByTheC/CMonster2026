package frc.robot.subsystems.shooter;

import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.epilogue.Logged;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.ShooterConstants.FlywheelConstants.*;
import static frc.robot.Constants.ShooterConstants.HoodConstants;

import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.sim.MechanismSim;
import frc.robot.sim.SimulationContext;

@Logged
public class SimShooterIO implements ShooterIO {
	@NotLogged private final FlywheelSim flywheelSimA;
	@NotLogged private final FlywheelSim flywheelSimB;
	@NotLogged private final MechanismSim flywheelMechanismSimA;
	@NotLogged private final MechanismSim flywheelMechanismSimB;

	@NotLogged private final DCMotorSim intermediarySim;
	@NotLogged private final MechanismSim intermediaryMechanismSim;

	@NotLogged private final SingleJointedArmSim hoodSim;
	@NotLogged private final MechanismSim hoodMechanismSim;

	private final SimpleMotorFeedforward flywheelFeedForward;
	private final PIDController flywheelPIDController;

	public SimShooterIO() {
		flywheelSimA = new FlywheelSim(
				LinearSystemId.createFlywheelSystem(DCMotor.getNEO(2).withReduction(1 / 1.5), 6 / 3417.2, 1),
				DCMotor.getNEO(2).withReduction(1 / 1.5));
		flywheelSimB = new FlywheelSim(
				LinearSystemId.createFlywheelSystem(DCMotor.getNEO(2).withReduction(1 / 1.5), 6 / 3417.2, 1),
				DCMotor.getNEO(2).withReduction(1 / 1.5));
		intermediarySim = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.1, 1),
				DCMotor.getNEO(1));
		hoodSim = new SingleJointedArmSim(DCMotor.getNEO(1), 60, 0.1, 0.3, 0, Math.PI, true, 0);

		flywheelMechanismSimA = new MechanismSim() {
			@Override
			public double getCurrentDraw() {
				return flywheelSimA.getCurrentDrawAmps();
			}

			@Override
			public void update(double timestep) {
				flywheelSimA.update(timestep);
			}
		};
		flywheelMechanismSimB = new MechanismSim() {
			@Override
			public double getCurrentDraw() {
				return flywheelSimB.getCurrentDrawAmps();
			}

			@Override
			public void update(double timestep) {
				flywheelSimB.update(timestep);
			}
		};
		intermediaryMechanismSim = new MechanismSim() {
			@Override
			public double getCurrentDraw() {
				return intermediarySim.getCurrentDrawAmps();
			}

			@Override
			public void update(double timestep) {
				intermediarySim.update(timestep);
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

		flywheelFeedForward = new SimpleMotorFeedforward(KS, KV);
		flywheelPIDController = new PIDController(KP, KI, KD);
		hoodPIDController = new PIDController(HoodConstants.KP, HoodConstants.KI, HoodConstants.KD);

		SimulationContext.getDefault().addMechanism(flywheelMechanismSimA);
		SimulationContext.getDefault().addMechanism(flywheelMechanismSimB);
		SimulationContext.getDefault().addMechanism(intermediaryMechanismSim);
		SimulationContext.getDefault().addMechanism(hoodMechanismSim);
	}

	@Override
	public void stopFlywheel() {
		flywheelSimA.setInputVoltage(0);
		flywheelSimB.setInputVoltage(0);
	}

	@Override
	public void stopIntermediary() {
		intermediarySim.setInputVoltage(0);
	}

	@Override
	public void stopHood() {
		hoodSim.setInputVoltage(0);
	}

	@Override
	public void setFlywheelVelocity(AngularVelocity angularVelocity) {
		double feedForward = flywheelFeedForward.calculate(angularVelocity.in(RPM));
		double pid = flywheelPIDController.calculate(flywheelSimA.getAngularVelocity().in(RPM), angularVelocity.in(RPM));

		flywheelSimA.setInputVoltage(flywheelMechanismSimA.outputVoltage(pid + feedForward));
		flywheelSimB.setInputVoltage(flywheelMechanismSimB.outputVoltage(pid + feedForward));
	}

	@Override
	public void setIntermediaryVoltage(Voltage voltage) {
		intermediarySim.setInputVoltage(intermediaryMechanismSim.outputVoltage(voltage.in(Volts)));
	}

	@Override
	public void setHoodAngle(Angle angle) {}

	@Override
	public AngularVelocity getFlywheelVelocity() {
		return flywheelSimA.getAngularVelocity().plus(flywheelSimB.getAngularVelocity()).div(2);
	}

	@Override
	public Current getCurrentDraw() {
		return Amps.of(flywheelSimA.getCurrentDrawAmps());
	}
}
