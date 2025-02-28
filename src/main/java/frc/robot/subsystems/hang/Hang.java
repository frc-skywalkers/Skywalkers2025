package frc.robot.subsystems.hang;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.HangConstants;
import frc.robot.Constants.Mode;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Hang extends SubsystemBase {
  private final HangIO io;
  private HangIOInputsAutoLogged inputs = new HangIOInputsAutoLogged();
  private final ArmFeedforward ffModel;

  public Hang(HangIO io) {
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
    Logger.processInputs(("Hang"), inputs);
    // Logger.recordOutput("Hang/atPosition", atPosition(setp));
  }
  ;

  public void runVolts(double volts) {
    io.setVoltage(volts);
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
    return Math.abs(getPositionRad() - inputs.goalPos) < HangConstants.tolerance;
  }

  public boolean atPosition(double goalPos) {
    boolean ret = Math.abs(getPositionRad() - goalPos) < HangConstants.tolerance;
    Logger.recordOutput("Hang/atPositions", ret);
    Logger.recordOutput("Hang/measuredGoalPos", getPositionRad());
    return ret;
  }

  public boolean atDropDown() {
    return Math.abs(getPositionRad() - HangConstants.dropDown) < HangConstants.downtolerance;
  }
}
