package frc.robot;

import com.revrobotics.REVLibError;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import java.util.function.Supplier;

@Logged
public class SparkPinger {
  public static SparkPinger INSTANCE;

  @NotLogged public Supplier<REVLibError> frontLeftDrive;
  @Logged private boolean frontLeftDriveConnected = false;
  @NotLogged public Supplier<REVLibError> frontLeftTurn;
  @Logged private boolean frontLeftTurnConnected = false;
  @NotLogged public Supplier<REVLibError> frontRightDrive;
  @Logged private boolean frontRightDriveConnected = false;
  @NotLogged public Supplier<REVLibError> frontRightTurn;
  @Logged private boolean frontRightTurnConnected = false;
  @NotLogged public Supplier<REVLibError> backLeftDrive;
  @Logged private boolean backLeftDriveConnected = false;
  @NotLogged public Supplier<REVLibError> backLeftTurn;
  @Logged private boolean backLeftTurnConnected = false;
  @NotLogged public Supplier<REVLibError> backRightDrive;
  @Logged private boolean backRightDriveConnected = false;
  @NotLogged public Supplier<REVLibError> backRightTurn;
  @Logged private boolean backRightTurnConnected = false;
  @NotLogged public Supplier<REVLibError> intakeWrist;
  @Logged private boolean intakeWristConnected = false;
  @NotLogged public Supplier<REVLibError> intakeRoller;
  @Logged private boolean intakeRollerConnected = false;
  @NotLogged public Supplier<REVLibError> rightFlywheelLeader;
  @Logged private boolean rightFlywheelLeaderConnected = false;
  @NotLogged public Supplier<REVLibError> rightFlywheelFollower;
  @Logged private boolean rightFlywheelFollowerConnected = false;
  @NotLogged public Supplier<REVLibError> leftFlywheelLeader;
  @Logged private boolean leftFlywheelLeaderConnected = false;
  @NotLogged public Supplier<REVLibError> leftFlywheelFollower;
  @Logged private boolean leftFlywheelFollowerConnected = false;
  @NotLogged public Supplier<REVLibError> leftFeeder;
  @Logged private boolean leftFeederConnected = false;
  @NotLogged public Supplier<REVLibError> rightFeeder;
  @Logged private boolean rightFeederConnected = false;
  @NotLogged public Supplier<REVLibError> hood;
  @Logged private boolean hoodConnected = false;
  @NotLogged public Supplier<REVLibError> hopper;
  @Logged private boolean hopperConnected = false;

  public SparkPinger() {
    INSTANCE = this;
  }

  public void periodicPing() {
    frontLeftDriveConnected = isConnected(frontLeftDrive);
    frontLeftTurnConnected = isConnected(frontLeftTurn);
    frontRightDriveConnected = isConnected(frontRightDrive);
    frontRightTurnConnected = isConnected(frontRightTurn);
    backLeftDriveConnected = isConnected(backLeftDrive);
    backLeftTurnConnected = isConnected(backLeftTurn);
    backRightDriveConnected = isConnected(backRightDrive);
    backRightTurnConnected = isConnected(backRightTurn);
    intakeWristConnected = isConnected(intakeWrist);
    intakeRollerConnected = isConnected(intakeRoller);
    rightFlywheelLeaderConnected = isConnected(rightFlywheelLeader);
    rightFlywheelFollowerConnected = isConnected(rightFlywheelFollower);
    leftFlywheelLeaderConnected = isConnected(leftFlywheelLeader);
    leftFlywheelFollowerConnected = isConnected(leftFlywheelFollower);
    leftFeederConnected = isConnected(leftFeeder);
    rightFeederConnected = isConnected(rightFeeder);
    hoodConnected = isConnected(hood);
    hopperConnected = isConnected(hopper);
  }

  private static boolean isConnected(Supplier<REVLibError> spark) {
    if (spark == null)
      return false;
    return spark.get().equals(REVLibError.kOk);
  }
}
