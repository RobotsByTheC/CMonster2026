package frc.robot.logging.epilogue;

import com.revrobotics.RelativeEncoder;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

@CustomLoggerFor(RelativeEncoder.class)
public class RelativeEncoderLogger extends ClassSpecificLogger<RelativeEncoder> {
  public RelativeEncoderLogger() {
    super(RelativeEncoder.class);
  }

  @Override
  protected void update(EpilogueBackend epilogueBackend, RelativeEncoder relativeEncoder) {
    //    epilogueBackend.log("position", relativeEncoder.getPosition());
    //    epilogueBackend.log("velocity", relativeEncoder.getVelocity());
  }
}
