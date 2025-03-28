package frc.robot.subsystems.coralintake;

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
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.CoralIntakeConstants;

public class CoralIntakeIOTalonFX implements CoralIntakeIO {
  // private double goalPos = 0.00;

  // private final TalonFX pivot = new TalonFX(CoralIntakeConstants.pivotID);
  private final TalonFX wheel = new TalonFX(CoralIntakeConstants.wheelID);

  private final DigitalInput beamBreak = new DigitalInput(CoralIntakeConstants.beamBreakID);

  // private final StatusSignal<Angle> position = pivot.getPosition();
  // private final StatusSignal<AngularVelocity> velocity = pivot.getVelocity();
  // private final StatusSignal<Voltage> appliedVolts = pivot.getMotorVoltage();
  // private final StatusSignal<Current> current = pivot.getTorqueCurrent();
  // private final StatusSignal<Double> goal = pivot.getClosedLoopReference();

  private final StatusSignal<AngularVelocity> wheelVelocity = wheel.getVelocity();
  private final StatusSignal<Voltage> wheelAppliedVolts = wheel.getMotorVoltage();
  private final StatusSignal<Current> wheelCurrent = wheel.getTorqueCurrent();

  // private double initialAbs = 0.0;
  // private final double absEncoderOffset = 0.0; // FIND THIS VALUE!!!!!!!!!!!!!
  // private double outputOffset = 0.0;

  // private final MotionMagicVoltage mm_volt = new MotionMagicVoltage(0.0); // for the pivot

  public CoralIntakeIOTalonFX() {
    // var pivotConfig = new TalonFXConfiguration();
    // pivotConfig.CurrentLimits.StatorCurrentLimit = 60.0;
    // pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    // pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    var wheelConfig = new TalonFXConfiguration();
    wheelConfig.CurrentLimits.StatorCurrentLimit = 50.0;
    wheelConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    wheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake; // ?

    // Slot0Configs pidConfigs = pivotConfig.Slot0;

    // needs to be changed
    // pidConfigs.GravityType = GravityTypeValue.Arm_Cosine;
    // pidConfigs.kS = 0.1;
    // pidConfigs.kG = 0.0; // 0.0
    // pidConfigs.kP = 25.0;
    // pidConfigs.kI = 0.0;
    // pidConfigs.kD = 0.06;
    // pidConfigs.kV = 0.12 * CoralIntakeConstants.GEAR_RATIO;
    // pidConfigs.kA = 0.045;

    // needs to be changed
    // MotionMagicConfigs mm_configs = pivotConfig.MotionMagic;
    // mm_configs.MotionMagicCruiseVelocity = CoralIntakeConstants.mm_cruisevel;
    // mm_configs.MotionMagicAcceleration = CoralIntakeConstants.mm_accel;
    // mm_configs.MotionMagicJerk = CoralIntakeConstants.mm_jerk; //this value is optional

    // FeedbackConfigs fdb_configs = pivotConfig.Feedback;
    // fdb_configs.SensorToMechanismRatio = CoralIntakeConstants.GEAR_RATIO;

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    // for (int i = 0; i < 5; ++i) {
    //   status = pivot.getConfigurator().apply(pivotConfig);
    //   if (status.isOK()) break;
    // }
    // if (!status.isOK()) {
    //   System.out.println("Real Error, Could not configure device. Error: " + status.toString());
    // }

    // status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = wheel.getConfigurator().apply(wheelConfig);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Real Error, Could not configure device. Error: " + status.toString());
    }

    // BaseStatusSignal.setUpdateFrequencyForAll(
    //     250.0, position, velocity, appliedVolts, current, goal);
    // pivot.optimizeBusUtilization();

    BaseStatusSignal.setUpdateFrequencyForAll(
        250.0, wheelVelocity, wheelAppliedVolts, wheelCurrent);
    wheel.optimizeBusUtilization();

    // pivot.setPosition(0); // so that 0 is horizontal

    // the starting position is zeroed - need to make sure we start at same spot each

    // pivot.setPosition(absEncoderPos.getValueAsDouble() - absEncoderOffset); //this one fr
    // outputOffset = initialAbs - absEncoderOffset;
  }

  @Override
  public void updateInputs(CoralIntakeIOInputs inputs) {
    // BaseStatusSignal.refreshAll(position, velocity, appliedVolts, current, goal);
    BaseStatusSignal.refreshAll(wheelVelocity, wheelAppliedVolts, wheelCurrent);

    // inputs.positionRad = Units.rotationsToRadians(position.getValueAsDouble());
    // inputs.velocityRadPerSec = Units.rotationsToRadians(velocity.getValueAsDouble());
    // inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps =
        new double[] {wheelCurrent.getValueAsDouble(), wheelCurrent.getValueAsDouble()};
    // inputs.goalPos = goalPos;
    // inputs.setpointPos = goal.getValueAsDouble();

    inputs.wheelVelocityRadPerSec = Units.rotationsToRadians(wheelVelocity.getValueAsDouble());
    inputs.wheelAppliedVolts = wheelAppliedVolts.getValueAsDouble();

    inputs.hasPiece = hasPiece();

    // SmartDashboard.putNumber("Coral pos", position.getValueAsDouble());
  }

  // @Override
  // public void setVoltage(double volts) {
  //   pivot.setControl(new VoltageOut(volts));
  // }

  // @Override
  // public void stop() {
  //   pivot.stopMotor();
  // }

  @Override
  public void runWheelVolts(double volts) {
    wheel.setControl(new VoltageOut(volts));
  }

  @Override
  public void stopWheels() {
    wheel.stopMotor();
  }

  // @Override
  // public void goToPosition(double positionRad) {
  //   goalPos = positionRad;
  //   pivot.setControl(mm_volt.withPosition(Units.radiansToRotations(positionRad)).withSlot(0));
  // }

  // @Override
  // public void resetPosition() {
  //   pivot.setPosition(0);
  // }

  public double getWheelCurrent() {
    return wheelCurrent.getValueAsDouble();
  }

  public boolean hasPiece() {
    return !beamBreak.get(); // beam broken = .get() is false = has Piece is true
  }
}
