package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import java.util.function.Supplier;

@Logged
public class Dashboard {
  @NotLogged private Supplier<Boolean> gyroSupplier;
  @NotLogged private Supplier<Boolean[]> swerveSupplier;

  public Dashboard(Supplier<Boolean> gyro, Supplier<Boolean[]> swerve) {
    gyroSupplier = gyro;
    swerveSupplier = swerve;
  }

  private boolean gyro = false;
  private boolean frontLeftDrive = false;
  private boolean frontLeftTurn = false;
  private boolean frontRightDrive = false;
  private boolean frontRightTurn = false;
  private boolean backLeftDrive = false;
  private boolean backLeftTurn = false;
  private boolean backRightDrive = false;
  private boolean backRightTurn = false;

  public void updateDashboard() {
    gyro = gyroSupplier.get();
    Boolean[] swerveStatus = swerveSupplier.get();
    frontLeftDrive = swerveStatus[0];
    frontLeftTurn = swerveStatus[1];
    frontRightDrive = swerveStatus[2];
    frontRightTurn = swerveStatus[3];
    backLeftDrive = swerveStatus[4];
    backLeftTurn = swerveStatus[5];
    backRightDrive = swerveStatus[6];
    backRightTurn = swerveStatus[7];
  }
}