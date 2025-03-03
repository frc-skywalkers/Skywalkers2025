package frc.robot.subsystems.hang;

import org.littletonrobotics.junction.AutoLog;

public interface HangIO {
  @AutoLog
  public static class HangIOInputs {
    // for the pivot
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }

  /* updates set of loggable inputs */
  public default void updateInputs(HangIOInputs inputs) {}

  /* run open loop at specified voltage */
  public default void setVoltage(double volts) {}

  /* stops in open loop */
  public default void stop() {}

  public default void resetPosition() {} // resetting encoder reading
}
