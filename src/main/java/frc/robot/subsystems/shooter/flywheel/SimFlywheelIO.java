package frc.robot.subsystems.shooter.flywheel;

import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.epilogue.Logged;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.ShooterConstants.FlywheelConstants;
import frc.robot.sim.MechanismSim;
import frc.robot.sim.SimulationContext;

@Logged
public class SimFlywheelIO implements FlywheelIO {
	@NotLogged private final FlywheelSim flywheelSim;
	@NotLogged private final MechanismSim flywheelMechanismSim;
	private final SimpleMotorFeedforward flywheelFeedForward;
	private final PIDController flywheelPIDController;

	private final MutAngularVelocity targetFlywheelSpeed = RPM.mutable(0);

	public SimFlywheelIO() {
		flywheelSim = new FlywheelSim(
				LinearSystemId.createFlywheelSystem(DCMotor.getNEO(2).withReduction(1 / 1.5), 6 / 3417.2, 1),
				DCMotor.getNEO(2).withReduction(1 / 1.5));

		flywheelMechanismSim = new MechanismSim() {
			@Override
			public double getCurrentDraw() {
				return flywheelSim.getCurrentDrawAmps();
			}

			@Override
			public void update(double timestep) {
				double feedForward = flywheelFeedForward.calculate(targetFlywheelSpeed.in(RPM));
				double pid = flywheelPIDController.calculate(flywheelSim.getAngularVelocity().in(RPM),
						targetFlywheelSpeed.in(RPM));
				flywheelSim.setInputVoltage(flywheelMechanismSim.outputVoltage(pid + feedForward));

				flywheelSim.update(timestep);
			}
		};

		flywheelFeedForward = new SimpleMotorFeedforward(FlywheelConstants.KS, FlywheelConstants.KV);
		flywheelPIDController = new PIDController(FlywheelConstants.KP, FlywheelConstants.KI, FlywheelConstants.KD);

		SimulationContext.getDefault().addMechanism(flywheelMechanismSim);
	}

	@Override
	public void stop() {
		targetFlywheelSpeed.mut_setMagnitude(0);
	}

	@Override
	public void setVelocity(AngularVelocity angularVelocity) {
		targetFlywheelSpeed.mut_setMagnitude(angularVelocity.in(RPM));
	}

	@Override
	public AngularVelocity getVelocity() {
		return flywheelSim.getAngularVelocity();
	}

	@Override
	public Current getCurrentDraw() {
		return Amps.of(flywheelSim.getCurrentDrawAmps());
	}
}
