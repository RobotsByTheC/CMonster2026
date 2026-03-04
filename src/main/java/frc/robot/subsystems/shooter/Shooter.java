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
  private final Flywheel leftFlywheel;
  private final Feeder leftFeeder;
  private final Flywheel rightFlywheel;
  private final Feeder rightFeeder;
  private final Hood hood;

  public Shooter(boolean real) {
    if (real) {
      hood = new Hood(new RealHoodIO());

      leftFlywheel = new Flywheel(new RealFlywheelIO(true, FLYWHEEL_LEFT_A_CAN_ID, FLYWHEEL_LEFT_B_CAN_ID,
          Constants.ShooterConstants.FlywheelConstants.LeftConstants.KP,
          Constants.ShooterConstants.FlywheelConstants.LeftConstants.KI,
          Constants.ShooterConstants.FlywheelConstants.LeftConstants.KD,
          Constants.ShooterConstants.FlywheelConstants.LeftConstants.KS,
          Constants.ShooterConstants.FlywheelConstants.LeftConstants.KV));
      leftFeeder = new Feeder(
          new RealFeederIO(false, FEEDER_LEFT_CAN_ID, LEFT_CNC_BOTTOM, LEFT_CNC_MIDDLE, LEFT_CNC_TOP),
          new Trigger(leftFlywheel::atTargetSpeed));
      rightFlywheel = new Flywheel(new RealFlywheelIO(true, FLYWHEEL_RIGHT_A_CAN_ID, FLYWHEEL_RIGHT_B_CAN_ID,
          Constants.ShooterConstants.FlywheelConstants.RightConstants.KP,
          Constants.ShooterConstants.FlywheelConstants.RightConstants.KI,
          Constants.ShooterConstants.FlywheelConstants.RightConstants.KD,
          Constants.ShooterConstants.FlywheelConstants.RightConstants.KS,
          Constants.ShooterConstants.FlywheelConstants.RightConstants.KV));
      rightFeeder = new Feeder(
          new RealFeederIO(true, FEEDER_RIGHT_CAN_ID, RIGHT_CNC_BOTTOM, RIGHT_CNC_MIDDLE, RIGHT_CNC_TOP),
          new Trigger(rightFlywheel::atTargetSpeed));

    } else {
      hood = new Hood(new SimHoodIO());
      leftFlywheel = new Flywheel(new SimFlywheelIO());
      leftFeeder = new Feeder(new SimFeederIO(), new Trigger(leftFlywheel::atTargetSpeed));
      rightFlywheel = new Flywheel(new SimFlywheelIO());
      rightFeeder = new Feeder(new SimFeederIO(), new Trigger(rightFlywheel::atTargetSpeed));
    }

    leftFlywheel.setDefaultCommand(leftFlywheel.f_idle());
    rightFlywheel.setDefaultCommand(rightFlywheel.f_idle());
    leftFeeder.setDefaultCommand(leftFeeder.o_stop());
    rightFeeder.setDefaultCommand(rightFeeder.o_stop());
    hood.setDefaultCommand(hood.o_stop());
  }

  public Command synchronizedRev(Supplier<AngularVelocity> velocity) {
    Command command = leftFlywheel.f_shoot(velocity).alongWith(rightFlywheel.f_shoot(velocity));
    command.addRequirements(this);
    return command;
  }

  public Command feed() {
    return leftFeeder.f_activate().alongWith(rightFeeder.f_activate());
  }

  public Command f_aimAndRev() {
    return synchronizedRev(LookupTable::getVelocity).alongWith(hood.f_holdDesiredAngle(LookupTable::getAngle));
  }

  public Command l_normalize_hood() {
    return hood.l_returnToNormalcy();
  }

  public Command l_kapow() {
    return leftFeeder.queueBall().alongWith(rightFeeder.queueBall());
  }
}
