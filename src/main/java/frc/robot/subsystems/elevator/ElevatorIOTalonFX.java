package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.ElevatorConstants;
import org.littletonrobotics.junction.Logger;

public class ElevatorIOTalonFX implements ElevatorIO {
  private double goalPos = 0.0;

  private final TalonFX leftElevator = new TalonFX(ElevatorConstants.leftElevatorID);
  private final TalonFX rightElevator = new TalonFX(ElevatorConstants.rightElevatorID);

  private final CANcoder cancoder = new CANcoder(ElevatorConstants.encoderID);

  private final StatusSignal<Angle> leftPosition = leftElevator.getPosition();
  private final StatusSignal<Angle> rightPosition = rightElevator.getPosition();
  private final StatusSignal<AngularVelocity> leftVelocity = leftElevator.getVelocity();
  private final StatusSignal<AngularVelocity> rightVelocity =
      rightElevator.getVelocity(); // just check they are following properly
  private final StatusSignal<Voltage> appliedVolts = leftElevator.getMotorVoltage();
  private final StatusSignal<Current> leftCurrent = leftElevator.getTorqueCurrent();
  private final StatusSignal<Current> rightCurrent = rightElevator.getTorqueCurrent();

  private final StatusSignal<Angle> absEncoderPos = cancoder.getAbsolutePosition();

  private double initialAbs = 0.0;
  private final double absEncoderOffset = 0.0; // FIND THIS VALUE!!!!!!!!!!!!!
  private double outputOffset = 0.0;

  private final MotionMagicVoltage mm_volt = new MotionMagicVoltage(0.0);

  public ElevatorIOTalonFX() {
    var config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit = 60.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake; // BRAKE

    Slot0Configs pidConfigs = config.Slot0;

    pidConfigs.GravityType = GravityTypeValue.Elevator_Static;
    pidConfigs.kS = 0.15; // check all these values
    pidConfigs.kG = 0.28;
    pidConfigs.kP = 30.0; // 20.0 0.03 rad
    pidConfigs.kI = 0.00;
    pidConfigs.kD = 3.0;
    pidConfigs.kV = 0.12 * ElevatorConstants.GEAR_RATIO; // 0.38731 //2.4
    pidConfigs.kA = 0.045;

    MotionMagicConfigs mm_configs = config.MotionMagic;
    mm_configs.MotionMagicCruiseVelocity = ElevatorConstants.mm_cruisevel; // the max velocity
    mm_configs.MotionMagicAcceleration = ElevatorConstants.mm_accel;
    // mm_configs.MotionMagicJerk = ElevatorConstants.mm_jerk;

    FeedbackConfigs fdb_configs = config.Feedback;
    fdb_configs.SensorToMechanismRatio = ElevatorConstants.GEAR_RATIO;

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = leftElevator.getConfigurator().apply(config);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Real Error, Could not configure device. Error: " + status.toString());
    }

    status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = rightElevator.getConfigurator().apply(config);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Real Error, Could not configure device. Error: " + status.toString());
    }

    cancoder.getConfigurator().apply(new CANcoderConfiguration());

    BaseStatusSignal.setUpdateFrequencyForAll(
        250.0,
        leftPosition,
        rightPosition,
        leftVelocity,
        rightVelocity,
        appliedVolts,
        leftCurrent,
        rightCurrent);
    leftElevator.optimizeBusUtilization();
    rightElevator.optimizeBusUtilization();

    leftElevator.setPosition(0.0);
    rightElevator.setPosition(0.0);

    // leftElevator.setPosition(absEncoderPos.getValueAsDouble() - absEncoderOffset); //this one fr
    // rightElevator.setPosition(absEncoderPos.getValueAsDouble() - absEncoderOffset);
    initialAbs = absEncoderPos.getValueAsDouble();
    outputOffset = initialAbs - absEncoderOffset;

    rightElevator.setControl(new Follower(ElevatorConstants.leftElevatorID, false));
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        leftPosition,
        rightPosition,
        leftVelocity,
        rightVelocity,
        appliedVolts,
        leftCurrent,
        rightCurrent);
    inputs.positionRad = Units.rotationsToRadians(leftPosition.getValueAsDouble());
    inputs.velocityRadPerSec = Units.rotationsToRadians(leftVelocity.getValueAsDouble());
    inputs.appliedVolts = Units.rotationsToRadians(appliedVolts.getValueAsDouble());
    inputs.currentAmps =
        new double[] {leftCurrent.getValueAsDouble(), rightCurrent.getValueAsDouble()};
    inputs.goalPos = goalPos;
    // System.out.println(mm_volt.getFeedForwardMeasure());
    // System.out.println(mm_volt.getPositionMeasure());

    Logger.recordOutput("Elevator/initialAbs", initialAbs);
    Logger.recordOutput("Elevator/outputOffset", outputOffset);
  }

  @Override
  public void setVoltage(double volts) {
    leftElevator.setControl(new VoltageOut(volts));
  }

  @Override
  public void stop() {
    leftElevator.stopMotor();
  }

  @Override
  public void goToPosition(double positionRad) {
    goalPos = positionRad;
    leftElevator.setControl(
        mm_volt.withPosition(Units.radiansToRotations(positionRad)).withSlot(0));
  }

  @Override
  public void resetPosition() {
    leftElevator.setPosition(0);
  }
}
