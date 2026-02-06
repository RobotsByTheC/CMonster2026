package frc.robot.subsystems;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

public class ConstantTuner {
  public static Command createRoutine(Consumer<Voltage> v, Subsystem sys, BooleanSupplier up, BooleanSupplier low) {
    SysIdRoutine routine = new SysIdRoutine(new SysIdRoutine.Config(Volts.per(Second).of(0.1), Volts.of(0.5), null),
        new SysIdRoutine.Mechanism(v, null, sys));
    return routine.dynamic(SysIdRoutine.Direction.kForward).until(up)
        .andThen(routine.dynamic(SysIdRoutine.Direction.kReverse).until(low))
        .andThen(routine.quasistatic(SysIdRoutine.Direction.kForward).until(up))
        .andThen(routine.quasistatic(SysIdRoutine.Direction.kReverse).until(low));
  }
}
