package frc.robot.subsystems.shooter;

import static frc.robot.Constants.CANConstants.FEEDER_LEFT_CAN_ID;
import static frc.robot.Constants.CANConstants.FEEDER_RIGHT_CAN_ID;
import static frc.robot.Constants.CANConstants.FLYWHEEL_LEFT_A_CAN_ID;
import static frc.robot.Constants.CANConstants.FLYWHEEL_LEFT_B_CAN_ID;
import static frc.robot.Constants.CANConstants.FLYWHEEL_RIGHT_A_CAN_ID;
import static frc.robot.Constants.CANConstants.FLYWHEEL_RIGHT_B_CAN_ID;
import static frc.robot.Constants.CANConstants.LEFT_CNC_BOTTOM;
import static frc.robot.Constants.CANConstants.LEFT_CNC_MIDDLE;
import static frc.robot.Constants.CANConstants.LEFT_CNC_TOP;
import static frc.robot.Constants.CANConstants.RIGHT_CNC_BOTTOM;
import static frc.robot.Constants.CANConstants.RIGHT_CNC_MIDDLE;
import static frc.robot.Constants.CANConstants.RIGHT_CNC_TOP;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.data.LookupTable;
import frc.robot.subsystems.shooter.feeder.Feeder;
import frc.robot.subsystems.shooter.feeder.RealFeederIO;
import frc.robot.subsystems.shooter.feeder.SimFeederIO;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.flywheel.RealFlywheelIO;
import frc.robot.subsystems.shooter.flywheel.SimFlywheelIO;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.hood.RealHoodIO;
import frc.robot.subsystems.shooter.hood.SimHoodIO;
import java.util.function.Supplier;

@Logged
public class Shooter extends SubsystemBase {
  private final Flywheel rightFlywheel;

  public Shooter(boolean real) {
    if (real) {
      rightFlywheel = new Flywheel(new RealFlywheelIO(true, FLYWHEEL_RIGHT_A_CAN_ID, FLYWHEEL_RIGHT_B_CAN_ID,
          Constants.ShooterConstants.FlywheelConstants.RightConstants.KP,
          Constants.ShooterConstants.FlywheelConstants.RightConstants.KI,
          Constants.ShooterConstants.FlywheelConstants.RightConstants.KD,
          Constants.ShooterConstants.FlywheelConstants.RightConstants.KS,
          Constants.ShooterConstants.FlywheelConstants.RightConstants.KV));
    } else {
      rightFlywheel = new Flywheel(new SimFlywheelIO());
    }

    rightFlywheel.setDefaultCommand(rightFlywheel.f_idle());

  }

  public Command synchronizedRev(Supplier<AngularVelocity> velocity) {
    Command command = rightFlywheel.f_shoot(velocity);
    command.addRequirements(this);
    return command;
  }

  public Command applyFlywheelVoltage(Supplier<Voltage> voltage) {
    return rightFlywheel.f_applyVoltage(voltage);
  }
}
