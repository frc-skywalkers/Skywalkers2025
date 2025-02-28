package frc.robot.subsystems.hang;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class HangIOTalonFX implements HangIO {
  private final TalonFX hang = new TalonFX(0);
  private static final double kP = 0.05;
  private static final double kI = 0.0;
  private static final double kD = 0.0;
  private static final double kS = 0.2;
  private static final double kG = 0.5;
  private static final double kV = 0.1;

  private final ArmFeedforward feedforward = new ArmFeedforward(kS, kG, kV);
  private final PIDController pidController = new PIDController(kP, kI, kD);

  private double targetPosition = 0.0;

  public HangIOTalonFX() {
    var hangConfig = new TalonFXConfiguration();
    hangConfig.CurrentLimits.StatorCurrentLimit = 60.0;
    hangConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    hangConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // feedforward = new ArmFeedforward(kS, kG, kV);

    // pidController = new PIDController(kP, kI, kD);
  }

  @Override
  public void setVoltage(double volts) {
    hang.setControl(new VoltageOut(volts));
  }

  @Override
  public void stop() {
    hang.stopMotor();
  }

  public void rotateHang(double speed, boolean enableHangArms) {
    if (!enableHangArms || Math.abs(speed) < 0.05) {
      speed = 0;
    }
    StatusSignal<Angle> currentHangPosition = hang.getPosition();

    double ffVoltsLeft =
        feedforward.calculate(Math.toRadians(currentHangPosition.getValueAsDouble()), speed);

    double pidOutputLeft =
        pidController.calculate(currentHangPosition.getValueAsDouble(), targetPosition);

    // Apply feedforward + PID output
    double finalLeftVoltage = pidOutputLeft + ffVoltsLeft;

    hang.setVoltage(-finalLeftVoltage);

    SmartDashboard.putNumber("Left Climber Voltage", finalLeftVoltage);
  }

  @Override
  public void resetPosition() {
    hang.setPosition(0);
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    pidController.setP(kP);
    pidController.setI(kI);
    pidController.setD(kD);
    pidController.setPID(kP, kI, kD);
  }
}
