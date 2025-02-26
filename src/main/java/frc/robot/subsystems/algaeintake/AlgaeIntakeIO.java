package frc.robot.subsystems.algaeintake;

import org.littletonrobotics.junction.AutoLog;

public interface AlgaeIntakeIO {
  @AutoLog
  public static class AlgaeIntakeIOInputs {
    // for the pivot
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double goalPos = 0.0; // end goal of trajectory
    public double setpointPos = 0.000; // setpoints in the trajectory

    // for both
    public double[] currentAmps = new double[] {};

    // for the wheels; position not needed
    public double wheelVelocityRadPerSec = 0.0;
    public double wheelAppliedVolts = 0.0;
  }

  // for pivot

  /* updates set of loggable inputs */
  public default void updateInputs(AlgaeIntakeIOInputs inputs) {}

  /* run open loop at specified voltage */
  public default void setVoltage(double volts) {}

  /* go to specified position */
  public default void goToPosition(double positionRad) {}

  /* stops in open loop */
  public default void stop() {}

  public default void resetPosition() {} // resetting encoder reading

  // for wheels

  public default void runWheelVolts(double volts) {}

  public default void stopWheels() {}
}
