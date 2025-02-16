package frc.robot.subsystems.Algaeintake;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AlgaeintakeConstants;
import frc.robot.Constants.Mode;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Algaeintake extends SubsystemBase {
  private final AlgaeintakeIO io;
  private AlgaeintakeIOInputsAutoLogged inputs = new AlgaeintakeIOInputsAutoLogged();
  private final ArmFeedforward ffModel;

  public Algaeintake(AlgaeintakeIO io) {
    this.io = io;
    switch (Constants.currentMode) {
      case REAL:
        ffModel = new ArmFeedforward(0, 0, 0);
        io.configurePID(0, 0, 0);
        break;
      case REPLAY:
        ffModel = new ArmFeedforward(0.1, 0.05, 0);
        io.configurePID(1.0, 0.0, 0.0);
        break;
      case SIM:
        ffModel = new ArmFeedforward(0.0, 0.1, 0.6); // need to determine
        io.configurePID(10.0, 0.0, 0.1); // need to determine
        break;
      default:
        ffModel = new ArmFeedforward(0.0, 0.0, 0);
        break;
    }
    Logger.recordOutput("Intake/Mode", 0.000);
    Logger.recordOutput("Intake/atPosition", false);
    Logger.recordOutput("Intake/atPositions", false);
    Logger.recordOutput("Intake/measuredGoalPos", 0.0);
  }

  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs(("Intake"), inputs);
    Logger.recordOutput("Intake/hasPiece", hasPiece());
    // Logger.recordOutput("Intake/atPosition", atPosition(IntakeConstants.handoff));
  }
  ;

  public void runVolts(double volts) {
    io.setVoltage(volts);
  }

  public void runWheelVolts(double volts) {
    io.runWheelVolts(volts);
  }

  public void runWheel() {
    runWheelVolts(AlgaeintakeConstants.intakeVolts);
  }

  public void runWheelDouble() {
    runWheelVolts(1.5 * AlgaeintakeConstants.intakeVolts);
  }

  public void runWheelHalf() {
    runWheelVolts(AlgaeintakeConstants.intakeVolts / 1.5);
  }

  public void holdPiece() {
    runWheelVolts(AlgaeintakeConstants.holdVolts);
  }

  public void outtakeWheel() {
    runWheelVolts(AlgaeintakeConstants.outtakeVolts);
  }

  public boolean hasPiece() {
    // return inputs.tofDistance > IntakeConstants.tofTolerance;
    // return inputs.currentAmps[1] > 25.0;
    if (Constants.currentMode == Mode.SIM) return true;
    return inputs.currentAmps[1] < (-35.0);
    // return false;
  }
  // public void resetPosition(double position) {

  // }
  public void setPosition(double positionRad) {
    if (Constants.currentMode == Mode.SIM) {
      io.setPosition(positionRad, ffModel.calculate(inputs.setpointPos, inputs.goalVel));
      Logger.recordOutput(
          "Intake/FFoutput", 0.2 * Math.cos(inputs.setpointPos) + 0.764 * inputs.goalVel);
      Logger.recordOutput("Intake/goalVel", inputs.goalVel);

    } else {
      io.setPosition(positionRad, ffModel.calculate(inputs.positionRad, inputs.goalVel));
    }
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

  public double getCharacterizationVelocity() {
    return inputs.velocityRadPerSec;
  }

  public boolean atPosition() {
    return Math.abs(getPositionRad() - inputs.goalPos) < AlgaeintakeConstants.tolerance;
  }

  public boolean atPosition(double goalPos) {
    boolean ret = Math.abs(getPositionRad() - goalPos) < AlgaeintakeConstants.tolerance;
    Logger.recordOutput("Intake/atPositions", ret);
    Logger.recordOutput("Intake/measuredGoalPos", getPositionRad());
    return ret;
  }

  public boolean atDropDown() {
    return Math.abs(getPositionRad() - AlgaeintakeConstants.dropDown)
        < AlgaeintakeConstants.downtolerance;
  }

  public boolean isHoned() {
    return inputs.currentAmps[0] < -25.0;
  }

  // public Supplier<Boolean> gotPieceFinished() {
  // }

}
