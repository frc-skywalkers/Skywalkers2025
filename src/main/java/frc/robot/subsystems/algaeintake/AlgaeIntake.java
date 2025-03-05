package frc.robot.subsystems.algaeintake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeIntakeConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class AlgaeIntake extends SubsystemBase {
  private final AlgaeIntakeIO io;
  private final AlgaeIntakeIOInputsAutoLogged inputs = new AlgaeIntakeIOInputsAutoLogged();

  public AlgaeIntake(AlgaeIntakeIO io) {
    this.io = io;

    Logger.recordOutput("AlgaeIntake/atPosition", false);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("AlgaeIntake", inputs);

    Logger.recordOutput("AlgaeIntake/hasPiece", hasPiece());

    System.out.println(Math.abs(getPositionRad() - inputs.goalPos));
  }

  public void runVolts(double volts) {
    io.setVoltage(volts);
  }

  public void runWheelVolts(double volts) {
    io.runWheelVolts(volts);
  }

  public void intakeAlgae() {
    runWheelVolts(AlgaeIntakeConstants.intakeVolts);
  }

  public void holdPiece() {
    runWheelVolts(AlgaeIntakeConstants.holdVolts);
  }

  public void outtakeAlgae() {
    runWheelVolts(AlgaeIntakeConstants.outtakeVolts);
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

  // public boolean atPosition() {
  //   boolean ret = Math.abs(getPositionRad() - inputs.goalPos) < AlgaeIntakeConstants.tolerance;
  //   Logger.recordOutput("AlgaeIntake/atPosition", ret);
  //   if (ret) {
  //     System.out.println("REACHED POSITION!!!");
  //   }
  //   return ret;
  // }

  public boolean atPosition(double pos) {
    boolean ret = Math.abs(getPositionRad() - pos) < AlgaeIntakeConstants.tolerance;
    Logger.recordOutput("AlgaeIntake/atPosition", ret);
    if (ret) {
      System.out.println("REACHED POSITION!!!");
    }
    return ret;
  }

  public boolean isHomed() {
    return inputs.currentAmps[0] < -25.0; // check
  }
}
