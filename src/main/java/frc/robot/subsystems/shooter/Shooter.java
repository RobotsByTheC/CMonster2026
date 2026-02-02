package frc.robot.subsystems.shooter;

import static frc.robot.Constants.CANConstants.FLYWHEEL_LEFT_A_CAN_ID;
import static frc.robot.Constants.CANConstants.FLYWHEEL_LEFT_B_CAN_ID;
import static frc.robot.Constants.CANConstants.FLYWHEEL_RIGHT_A_CAN_ID;
import static frc.robot.Constants.CANConstants.FLYWHEEL_RIGHT_B_CAN_ID;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.data.LookupTable;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.flywheel.RealFlywheelIO;
import frc.robot.subsystems.shooter.flywheel.SimFlywheelIO;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.hood.RealHoodIO;
import frc.robot.subsystems.shooter.hood.SimHoodIO;
import java.util.function.Supplier;

public class Shooter extends SubsystemBase {
	private final Flywheel leftFlywheel;
	private final Flywheel rightFlywheel;
	private final Hood hood;

	public Shooter(boolean real) {
		if (real) {
			leftFlywheel = new Flywheel(new RealFlywheelIO(false, FLYWHEEL_LEFT_A_CAN_ID, FLYWHEEL_LEFT_B_CAN_ID));
			rightFlywheel = new Flywheel(new RealFlywheelIO(true, FLYWHEEL_RIGHT_A_CAN_ID, FLYWHEEL_RIGHT_B_CAN_ID));
			hood = new Hood(new RealHoodIO());
		} else {
			leftFlywheel = new Flywheel(new SimFlywheelIO());
			rightFlywheel = new Flywheel(new SimFlywheelIO());
			hood = new Hood(new SimHoodIO());
		}

		leftFlywheel.setDefaultCommand(leftFlywheel.f_idle());
		rightFlywheel.setDefaultCommand(rightFlywheel.f_idle());
		hood.setDefaultCommand(hood.f_idle());
	}

	public Command synchronizedRev(Supplier<AngularVelocity> velocity) {
		return leftFlywheel.f_shoot(velocity).alongWith(rightFlywheel.f_shoot(velocity));
	}

	public Command f_idle() {
		return leftFlywheel.f_shoot(() -> Constants.ShooterConstants.FlywheelConstants.IDLE_SPEED)
				.alongWith(rightFlywheel.f_shoot(() -> Constants.ShooterConstants.FlywheelConstants.IDLE_SPEED))
				.alongWith(hood.l_returnToNormalcy().andThen(hood.o_stop()));
	}

	public Command f_aimAndRev() {
		return synchronizedRev(LookupTable::getVelocity).alongWith(hood.f_holdDesiredAngle(LookupTable::getAngle));
	}
}
