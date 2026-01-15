package frc.robot.subsystems.shooter;

import edu.wpi.first.units.measure.AngularVelocity;

public interface ShooterIO {

    void stop();

    void setSpeed(AngularVelocity angularVelocity);

}