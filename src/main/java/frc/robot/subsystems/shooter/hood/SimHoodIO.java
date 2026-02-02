package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.sim.MechanismSim;
import frc.robot.sim.SimulationContext;

import static edu.wpi.first.units.Units.*;

@Logged
public class SimHoodIO implements HoodIO {
	@NotLogged private final SingleJointedArmSim sim;
	@NotLogged private final MechanismSim mechanism;

	public SimHoodIO() {
		sim = new SingleJointedArmSim(DCMotor.getNEO(1), 58.9362825, 0.003122, .1921, 0, Math.PI * 4, true, 0);
		mechanism = new MechanismSim() {
			@Override
			public double getCurrentDraw() {
				return sim.getCurrentDrawAmps();
			}

			@Override
			public void update(double timestep) {
				sim.update(timestep);
			}
		};

		SimulationContext.getDefault().addMechanism(mechanism);
	}

	@Override
	public Angle getAngle() {
		return Radians.of(sim.getAngleRads());
	}

	@Override
	public AngularVelocity getVelocity() {
		return RadiansPerSecond.of(sim.getVelocityRadPerSec());
	}

	@Override
	public void setVoltage(Voltage voltage) {
		sim.setInputVoltage(mechanism.outputVoltage(voltage.in(Volts)));
	}
}
