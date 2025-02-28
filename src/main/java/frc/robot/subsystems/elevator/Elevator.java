package frc.robot.subsystems.elevator;
// isZeroed

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  public Elevator(ElevatorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    // soft limits
    if (getPositionRad() > ElevatorConstants.kTopLimit && inputs.appliedVolts > 0) {
      stop();
    }
    if (getPositionRad() < ElevatorConstants.kBottomLimit && inputs.appliedVolts < 0) {
      stop();
    }
  }

  public void runVolts(double volts) {
    io.setVoltage(volts);
  }

  public void goToPosition(double positionRad) {
    io.goToPosition(positionRad);
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

  public boolean atPosition() {
    boolean ret = Math.abs(getPositionRad() - inputs.goalPos) < ElevatorConstants.tolerance;
    Logger.recordOutput("Elevator/atPosition", ret);
    return ret;
  }

  public boolean atPosition(double pos) {
    return Math.abs(getPositionRad() - pos) < ElevatorConstants.tolerance;
  }

  public boolean isHomed() {
    return inputs.currentAmps[0] < -25.0; // check
  }
}
