package frc.robot.subsystems.Algaeintake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeintakeConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Algaeintake extends SubsystemBase {
  private final AlgaeintakeIO io;
  private final AlgaeintakeIOInputsAutoLogged inputs = new AlgaeintakeIOInputsAutoLogged();

  public Algaeintake(AlgaeintakeIO io) {
    this.io = io;

    Logger.recordOutput("Algaeintake/atPosition", false);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Algaeintake", inputs);

    Logger.recordOutput("Algaeintake/hasPiece", hasPiece());
  }

  public void runVolts(double volts) {
    io.setVoltage(volts);
  }

  public void runWheelVolts(double volts) {
    io.runWheelVolts(volts);
  }

  public void intakeAlgae() {
    runWheelVolts(AlgaeintakeConstants.intakeVolts);
  }

  public void holdPiece() {
    runWheelVolts(AlgaeintakeConstants.holdVolts);
  }

  public void outtakeAlgae() {
    runWheelVolts(AlgaeintakeConstants.outtakeVolts);
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

  public void stopWheels() {
    io.stopWheels();
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

  public boolean hasPiece() {
    // if (Constants.currentMode == Mode.SIM) return true;
    return inputs.currentAmps[1] > (35.0); // check sign + amps number
  }

  public boolean atPosition() {
    boolean ret = Math.abs(getPositionRad() - inputs.goalPos) < AlgaeintakeConstants.tolerance;
    Logger.recordOutput("Algaeintake/atPosition", ret);
    return ret;
  }

  public boolean atPosition(double pos) {
    return Math.abs(getPositionRad() - pos) < AlgaeintakeConstants.tolerance;
  }

  public boolean isHomed() {
    return inputs.currentAmps[0] < -25.0; // check
  }
}
