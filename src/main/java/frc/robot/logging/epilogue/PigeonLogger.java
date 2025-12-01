package frc.robot.logging.epilogue;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

@CustomLoggerFor(Pigeon2.class)
public class PigeonLogger extends ClassSpecificLogger<Pigeon2> {
  public PigeonLogger() {
    super(Pigeon2.class);
  }

  @Override
  protected void update(EpilogueBackend backend, Pigeon2 object) {
    //    backend.log("Rotation", object.getRotation3d(), Rotation3d.struct);
    //    backend.log("Acceleration X", object.getAccelerationX().getValue());
    //    backend.log("Acceleration Y", object.getAccelerationY().getValue());
    //    backend.log("Acceleration Z", object.getAccelerationZ().getValue());
  }
}
