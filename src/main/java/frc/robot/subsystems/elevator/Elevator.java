package frc.robot.subsystems.elevator;
// isZeroed

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  // public boolean isZeroed = false; // should start as false when actually testing
  // private boolean softLimitsEnabled = false; // should start as false when actually testing

  public Elevator(ElevatorIO io) {
    this.io = io;

    Logger.recordOutput("Elevator/atPosition", false);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    Logger.recordOutput("Elevator/isHomed", isHomed());
    ;
    // soft limits
    // Logger.recordOutput("Elevator/softLimitsEnabled", softLimitsEnabled);
    // Logger.recordOutput("Elevator/isZeroed", isZeroed);

    // if (softLimitsEnabled) {
    if (getPositionRad() > ElevatorConstants.kTopLimit && inputs.appliedVolts > 0) {
      stop();
    }
    if (getPositionRad() < ElevatorConstants.kBottomLimit && inputs.appliedVolts < 0) {
      stop();
    }
    // }
  }

  public void runVolts(double volts) {
    io.setVoltage(volts);
  }

  public void goToPosition(double positionRad) {
    // if (isZeroed) {
    io.goToPosition(positionRad);
    // System.out.println("going");
    // }
  }

  public void resetPosition() {
    io.resetPosition();
  }

  public void stop() {
    io.stop();
  }

  @AutoLogOutput
  public double getVelocityRad() {
    return inputs.velocityRadPerSec;
  }

  @AutoLogOutput
  public double getPositionRad() {
    return inputs.positionRad;
  }

  public double getCharacterizationVelocity() { // ?
    return inputs.velocityRadPerSec;
  }

  // public boolean atPosition() {
  //   boolean ret = Math.abs(getPositionRad() - inputs.goalPos) < ElevatorConstants.tolerance;
  //   Logger.recordOutput("Elevator/atPosition", ret);
  //   System.out.println("FINISHED COMMAND!!!!!");
  //   return ret;
  // }

  public boolean atPosition(double pos) {
    boolean ret = Math.abs(getPositionRad() - pos) < ElevatorConstants.tolerance;
    Logger.recordOutput("Elevator/atPosition", ret);
    return ret;
  }

  public boolean isHomed() {
    return inputs.currentAmps[0] < -20.0; // check, was -18
  }

  // public void enableSoftLimits() {
  //   softLimitsEnabled = true;
  // }

  // public void disableSoftLimits() {
  //   softLimitsEnabled = false;
  // }
}
