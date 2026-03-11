package frc.robot.revroboticshacks;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.jni.CANSparkJNI;
import com.revrobotics.spark.SparkMax;
import java.lang.invoke.MethodHandle;
import java.lang.invoke.MethodHandles;
import java.lang.invoke.MethodType;
import java.lang.invoke.VarHandle;

public class HackedSparkMaxAlternateEncoder implements RelativeEncoder {
  // package-private to the revrobotics package
  final SparkMax sparkMax;
  final long sparkHandle;

  private static final VarHandle SPARK_HANDLE;
  private static final MethodHandle SPARKMAX_THROW_IF_CLOSED;

  static {
    try {
      MethodHandles.Lookup lookup = MethodHandles.privateLookupIn(SparkMax.class, MethodHandles.lookup());
      SPARK_HANDLE = lookup.findVarHandle(SparkMax.class, "sparkHandle", long.class);
      SPARKMAX_THROW_IF_CLOSED = lookup.findVirtual(SparkMax.class, "throwIfClosed", MethodType.methodType(void.class));
    } catch (NoSuchMethodException | NoSuchFieldException | IllegalAccessException e) {
      throw new Error(e);
    }
  }

  public HackedSparkMaxAlternateEncoder(SparkMax sparkMax) {
    this.sparkMax = sparkMax;

    this.sparkHandle = (long) SPARK_HANDLE.get(sparkMax);

    // If we ever add additional entries to the AlternateEncoderType enum, we need to use the
    // encoderType param.

    // let the driver check if sim is running and create sim if necessary
    CANSparkJNI.c_Spark_CreateAlternateEncoderSim(sparkHandle);
  }

  public double getPosition() {
    throwIfClosed();
    return CANSparkJNI.c_Spark_GetAltEncoderPosition(sparkHandle);
  }

  public double getVelocity() {
    throwIfClosed();
    return CANSparkJNI.c_Spark_GetAltEncoderVelocity(sparkHandle);
  }

  public REVLibError setPosition(double position) {
    throwIfClosed();
    return REVLibError.fromInt(CANSparkJNI.c_Spark_SetAltEncoderPosition(sparkHandle, (float) position));
  }

  private void throwIfClosed() {
    if (SPARKMAX_THROW_IF_CLOSED != null) {
      try {
        SPARKMAX_THROW_IF_CLOSED.invoke(sparkMax);
      } catch (Throwable t) {
      }
    }
  }
}
