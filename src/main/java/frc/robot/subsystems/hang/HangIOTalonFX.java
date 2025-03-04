package frc.robot.subsystems.hang;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.HangConstants;
import org.littletonrobotics.junction.Logger;

public class HangIOTalonFX implements HangIO {
  private double goalPos = 0.0;

  private final TalonFX hang = new TalonFX(HangConstants.hangID);
  private final CANcoder cancoder = new CANcoder(HangConstants.encoderID);

  private final StatusSignal<Angle> position = hang.getPosition();
  private final StatusSignal<AngularVelocity> velocity = hang.getVelocity();
  private final StatusSignal<Current> current = hang.getTorqueCurrent();
  private final StatusSignal<Voltage> appliedVolts = hang.getMotorVoltage();

  private final StatusSignal<Angle> absEncoderPos = cancoder.getAbsolutePosition();
  private double initialAbs = 0.0;
  private final double absEncoderOffset = 0.0; // FIND THIS VALUE!!!!!!!!!!!!!
  private double outputOffset = 0.0;

  private final MotionMagicVoltage mm_volt = new MotionMagicVoltage(0.0);

  public HangIOTalonFX() {
    var config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit = 60.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    Slot0Configs pidConfigs = config.Slot0;
    pidConfigs.kS = 0.1;
    pidConfigs.kP = 10.0;
    pidConfigs.kI = 0.0;
    pidConfigs.kD = 0.20;
    pidConfigs.kV = 0.12 * HangConstants.GEAR_RATIO;
    pidConfigs.kA = 0.25;

    MotionMagicConfigs mm_configs = config.MotionMagic;
    mm_configs.MotionMagicCruiseVelocity = HangConstants.mm_cruisevel;
    mm_configs.MotionMagicAcceleration = HangConstants.mm_accel;
    mm_configs.MotionMagicJerk = HangConstants.mm_jerk;

    FeedbackConfigs fdb_configs = config.Feedback;

    fdb_configs.SensorToMechanismRatio = HangConstants.GEAR_RATIO;

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = hang.getConfigurator().apply(config);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Real Error, Could not configure device. Error: " + status.toString());
    }

    cancoder.getConfigurator().apply(new CANcoderConfiguration());

    hang.setPosition(0.0);
    // hang.setPosition(absEncoderPos.getValueAsDouble() - absEncoderOffset); //this one fr
    initialAbs = absEncoderPos.getValueAsDouble();
    outputOffset = initialAbs - absEncoderOffset;

    BaseStatusSignal.setUpdateFrequencyForAll(250.0, position, current, velocity, appliedVolts);
    hang.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(HangIOInputs inputs) {
    BaseStatusSignal.refreshAll(position, current, velocity, appliedVolts);
    inputs.positionRad = Units.rotationsToRadians(position.getValueAsDouble());
    inputs.velocityRadPerSec = Units.rotationsToRadians(velocity.getValueAsDouble());
    inputs.appliedVolts = Units.rotationsToRadians(appliedVolts.getValueAsDouble());
    inputs.currentAmps = current.getValueAsDouble();
    inputs.goalPos = goalPos;

    Logger.recordOutput("Hang/initialAbs", initialAbs);
    Logger.recordOutput("Hang/outputOffset", outputOffset);
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

  @Override
  public void goToPosition(double positionRad) {
    goalPos = positionRad;
    hang.setControl(mm_volt.withPosition(Units.radiansToRotations(positionRad)).withSlot(0));
  }
}
