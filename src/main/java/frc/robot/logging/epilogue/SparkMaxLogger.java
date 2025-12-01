package frc.robot.logging.epilogue;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

@CustomLoggerFor(SparkMax.class)
public class SparkMaxLogger extends ClassSpecificLogger<SparkMax> {
  public SparkMaxLogger() {
    super(SparkMax.class);
  }

  @Override
  protected void update(EpilogueBackend backend, SparkMax sparkMax) {
    //    backend.log("appliedOutput", sparkMax.getAppliedOutput());
    //    backend.log("busVoltage", sparkMax.getBusVoltage());
    //    backend.log("hasActiveFault", sparkMax.hasActiveFault());
    //    backend.log("hasActiveWarning", sparkMax.hasActiveWarning());
    //    backend.log("outputCurrent", sparkMax.getOutputCurrent());
    //    backend.log("canID", sparkMax.getDeviceId());
    //    backend.log("firmware", sparkMax.getFirmwareString());
    //    backend.log("motorTemperature", sparkMax.getMotorTemperature());
  }
}
