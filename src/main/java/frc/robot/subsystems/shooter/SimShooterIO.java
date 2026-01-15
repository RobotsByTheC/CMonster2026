package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import static edu.wpi.first.units.Units.RPM;

public class SimShooterIO implements ShooterIO{

    private final FlywheelSim flywheelSim = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(
            DCMotor.getNEO(2).withReduction(1 / 1.5),
            6 / 3417.2, // convert 6 lb-in^2 to kg-m^2
            1
        ),
        DCMotor.getNEO(2).withReduction(1 / 1.5)
    );
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, 0.25);
    private final PIDController pidController = new PIDController(0, 0, 0);

    @Override
    public void stop() {
        flywheelSim.setInputVoltage(0);
    }

    @Override
    public void setSpeed(AngularVelocity angularVelocity) {
        double feedforwardVolts = feedforward.calculate(angularVelocity.in(RPM));
        double pidVolts = pidController.calculate(angularVelocity.in(RPM), flywheelSim.getAngularVelocity().in(RPM));
        double totalVolts = feedforwardVolts + pidVolts;
        flywheelSim.setInputVoltage(totalVolts);
    }

    // In SimShooterIO.java:
    public void updateSim() {
        flywheelSim.update(0.02); // 0.02 second update period
    }
}
