package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.epilogue.Logged;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static frc.robot.Constants.ShooterConstants.KD;
import static frc.robot.Constants.ShooterConstants.KI;
import static frc.robot.Constants.ShooterConstants.KP;
import static frc.robot.Constants.ShooterConstants.KS;
import static frc.robot.Constants.ShooterConstants.KV;

import frc.robot.sim.MechanismSim;
import frc.robot.sim.SimulationContext;

@Logged
public class SimShooterIO implements ShooterIO {
	private final FlywheelSim flywheelSim;
	private final MechanismSim flywheelMechanismSim;

	private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(KS, KV);
	private final PIDController pidController = new PIDController(KP, KI, KD);

	public SimShooterIO() {
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
				flywheelSim.update(timestep);
			}
		};

		SimulationContext.getDefault().addMechanism(flywheelMechanismSim);
	}

	@Override
	public void stop() {
		flywheelSim.setInputVoltage(0);
	}

	@Override
	public void setDesiredVelocity(AngularVelocity angularVelocity) {
		flywheelSim.setInputVoltage(flywheelMechanismSim.outputVoltage(feedforward.calculate(angularVelocity.in(RPM))
				+ pidController.calculate(flywheelSim.getAngularVelocity().in(RPM), angularVelocity.in(RPM))));
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
