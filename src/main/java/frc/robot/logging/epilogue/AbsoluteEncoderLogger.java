package frc.robot.logging.epilogue;

import com.revrobotics.AbsoluteEncoder;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

@CustomLoggerFor(AbsoluteEncoder.class)
public class AbsoluteEncoderLogger extends ClassSpecificLogger<AbsoluteEncoder> {
  public AbsoluteEncoderLogger() {
    super(AbsoluteEncoder.class);
  }

  @Override
  protected void update(EpilogueBackend epilogueBackend, AbsoluteEncoder absoluteEncoder) {
    //    epilogueBackend.log("position", absoluteEncoder.getPosition());
    //    epilogueBackend.log("velocity", absoluteEncoder.getVelocity());
  }
}
