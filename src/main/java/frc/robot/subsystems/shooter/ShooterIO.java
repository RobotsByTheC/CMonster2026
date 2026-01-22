package frc.robot.subsystems.shooter;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.AngularVelocity;

@Logged
public interface ShooterIO {

    void stop();

    void setSpeed(AngularVelocity angularVelocity);

    AngularVelocity getSpeed();

}