package frc.robot.subsystems.hang;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class HangIOTalonFX implements HangIO {
  private final TalonFX hang = new TalonFX(22);
  private final StatusSignal<Current> current = hang.getTorqueCurrent();
  private final StatusSignal<AngularVelocity> velocity =
      hang.getVelocity(); // in rotations per second
  private final StatusSignal<Voltage> appliedVolts = hang.getMotorVoltage();

  public HangIOTalonFX() {
    var config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit = 60.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = hang.getConfigurator().apply(config);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Real Error, Could not configure device. Error: " + status.toString());
    }

    BaseStatusSignal.setUpdateFrequencyForAll(250.0, current, velocity, appliedVolts);
    hang.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(HangIOInputs inputs) {
    BaseStatusSignal.refreshAll(current, velocity, appliedVolts);
    inputs.velocityRadPerSec = Units.rotationsToRadians(velocity.getValueAsDouble());
    inputs.appliedVolts = Units.rotationsToRadians(appliedVolts.getValueAsDouble());
    inputs.currentAmps = current.getValueAsDouble();
  }

  @Override
  public void setVoltage(double volts) {
    hang.setControl(new VoltageOut(volts));
  }

  @Override
  public void stop() {
    hang.stopMotor();
  }

  @Override
  public void resetPosition() {
    hang.setPosition(0);
  }
}
