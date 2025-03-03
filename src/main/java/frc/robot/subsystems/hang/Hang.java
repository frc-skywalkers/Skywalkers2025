package frc.robot.subsystems.hang;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
