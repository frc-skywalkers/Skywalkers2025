package frc.robot.subsystems.hang;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.HangConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Hang extends SubsystemBase {
  private final HangIO io;
  private HangIOInputsAutoLogged inputs = new HangIOInputsAutoLogged();

  public Hang(HangIO io) {
    this.io = io;
    Logger.recordOutput("Hang/appliedVolts", inputs.appliedVolts);
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

  public void goToPosition(double positionRad) {
    io.goToPosition(positionRad);
  }

  @AutoLogOutput
  public double getPositionRad() {
    return inputs.positionRad;
  }

  public boolean atPosition() {
    boolean ret = Math.abs(getPositionRad() - inputs.goalPos) < HangConstants.tolerance;
    Logger.recordOutput("Hang/atPosition", ret);
    // System.out.println("FINISHED COMMAND!!!!!");
    return ret;
  }

  public boolean atPosition(double pos) {
    return Math.abs(getPositionRad() - pos) < ElevatorConstants.tolerance;
  }

  public void stop() {
    io.stop();
  }

  @AutoLogOutput
  public double getVelocityRad() {
    return inputs.velocityRadPerSec;
  }

  @AutoLogOutput
  public double getCurrent() {
    return inputs.currentAmps;
  }

  @AutoLogOutput
  public double getVoltage() {
    return inputs.appliedVolts;
  }
}
