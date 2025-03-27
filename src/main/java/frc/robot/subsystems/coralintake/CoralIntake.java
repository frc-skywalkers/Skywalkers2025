package frc.robot.subsystems.coralintake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralIntakeConstants;
import org.littletonrobotics.junction.Logger;

public class CoralIntake extends SubsystemBase {
  private final CoralIntakeIO io;
  private final CoralIntakeIOInputsAutoLogged inputs = new CoralIntakeIOInputsAutoLogged();

  public CoralIntake(CoralIntakeIO io) {
    this.io = io;

    // Logger.recordOutput("CoralIntake/atPosition", false);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("CoralIntake", inputs);

    Logger.recordOutput("CoralIntake/hasPiece", inputs.hasPiece);
  }

  // public void runVolts(double volts) {
  //   io.setVoltage(volts);
  // }

  public void runWheelVolts(double volts) {
    io.runWheelVolts(volts);
  }

  public void intakeCoral() {
    runWheelVolts(CoralIntakeConstants.intakeVolts);
  }

  public void holdPiece() {
    runWheelVolts(CoralIntakeConstants.holdVolts);
  }

  public void outtakeCoral() {
    runWheelVolts(CoralIntakeConstants.outtakeVolts);
  }

  // public void goToPosition(double positionRad) {
  //   io.goToPosition(positionRad);
  // }

  // public void resetPosition() {
  //   io.resetPosition();
  // }

  // public void stop() {
  //   io.stop();
  // }

  public void stopWheels() {
    io.stopWheels();
  }

  // @AutoLogOutput
  // public double getVelocityRad() {
  //   return inputs.velocityRadPerSec;
  // }

  // @AutoLogOutput
  // public double getPositionRad() {
  //   return inputs.positionRad;
  // }

  // public double getCharacterizationVelocity() { // ?
  //   return inputs.velocityRadPerSec;
  // }

  public boolean hasPiece() {
    // if (Constants.currentMode == Mode.SIM) return true;
    // return inputs.currentAmps[1] > (45.0); // check sign + amps number
    return inputs.hasPiece;
  }

  // public boolean atPosition() {
  //   boolean ret = Math.abs(getPositionRad() - inputs.goalPos) < CoralIntakeConstants.tolerance;
  //   Logger.recordOutput("CoralIntake/atPosition", ret);
  //   return ret;
  // }

  // public boolean atPosition(double pos) {
  //   return Math.abs(getPositionRad() - pos) < CoralIntakeConstants.tolerance;
  // }

  // public boolean isHomed() {
  //   return inputs.currentAmps[0] < -25.0; // check
  // }
}
