package frc.robot.subsystems.shooter;

import java.util.function.Supplier;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase{
    private final ShooterIO io;
    public Shooter(ShooterIO io) {
        this.io = io;
        setDefaultCommand(stop());
    }
    public Command stop(){
        return run(
            () -> io.stop()
        ).withName("ShooterStop");
    }

    public Command runAtSpeed(Supplier<AngularVelocity> speedsupplier){
        return run(
            () -> io.setSpeed(
                speedsupplier.get()
            )
        ).withName("ShooterRunAtSpeed");
    }

}
