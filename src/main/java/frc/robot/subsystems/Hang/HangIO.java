package frc.robot.subsystems.Hang;

import org.littletonrobotics.junction.AutoLog;

public interface HangIO {

  @AutoLog
  public static class HangIOInputs {
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double[] currentAmps = new double[] {};
    public double goalPos = 0.0;
    public double goalVel = 0.0;
    public double setpointPos = 0.000;
    public double ffVolts = 0.0000;
    public double pidOut = 0.000;
  }

  public default void updateInputs(HangIOInputs inputs) {}

  public static void setVelocity(double velocity) {}
  /** Run open loop at the specified voltage. */
  public default void setVoltage(double volts) {}

  /** Run closed loop at the specified velocity. */
  public default void setPosition(double positionRad, double ffVolts) {}

  /** Stop in open loop. */
  public default void stop() {}

  public default void resetPosition() {}

  public default void rotateHang(double speed, boolean enableHangArms) {}

  /** Set velocity PID constants. */
  public default void configurePID(double kP, double kI, double kD) {}
}
