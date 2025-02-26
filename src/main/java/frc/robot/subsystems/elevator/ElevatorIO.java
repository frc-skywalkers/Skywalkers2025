package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double[] currentAmps = new double[] {};
    public double goalPos = 0.0;
  }

  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default void setVoltage(double volts) {}

  public default void goToPosition(double positionRad) {}

  public default void stop() {}

  public default void resetPosition() {}
}
