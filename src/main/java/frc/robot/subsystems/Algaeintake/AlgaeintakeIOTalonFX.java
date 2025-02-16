package frc.robot.subsystems.Algaeintake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AlgaeintakeConstants;

public class AlgaeintakeIOTalonFX implements AlgaeintakeIO {
  private static final double GEAR_RATIO = 3.0 * 4.0 * 2.0;
  private double goalPos = 0.00;
  private double goalVel = 0.0;

  private final TalonFX arm = new TalonFX(55);
  private final TalonFX wheel = new TalonFX(23);

  private final ProfiledPIDController pidd =
      new ProfiledPIDController(
          0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(2 * Math.PI, 1.75 * Math.PI));

  private final StatusSignal<Angle> armPosition = arm.getPosition();
  private final StatusSignal<AngularVelocity> armVelocity = arm.getVelocity();
  private final StatusSignal<Voltage> armAppliedVolts = arm.getMotorVoltage();
  private final StatusSignal<Current> armCurrent = arm.getTorqueCurrent();
  private final StatusSignal<Double> armGoal = arm.getClosedLoopReference();

  private final StatusSignal<AngularVelocity> wheelVelocity = wheel.getVelocity();
  private final StatusSignal<Voltage> wheelAppliedVolts = wheel.getMotorVoltage();
  private final StatusSignal<Current> wheelCurrent = wheel.getTorqueCurrent();

  private final MotionMagicVoltage mm_volt = new MotionMagicVoltage(0);

  private LinearFilter intakenPiece = LinearFilter.movingAverage(5);

  public AlgaeintakeIOTalonFX() {
    var armConfig = new TalonFXConfiguration();
    armConfig.CurrentLimits.StatorCurrentLimit = 60.0;
    armConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    armConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    var wheelConfig = new TalonFXConfiguration();
    wheelConfig.CurrentLimits.StatorCurrentLimit = 50.0;
    wheelConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    wheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    Slot0Configs pidConfigs = armConfig.Slot0;

    pidConfigs.GravityType = GravityTypeValue.Arm_Cosine;
    pidConfigs.kS = 0.1;
    pidConfigs.kG = 0.2;
    pidConfigs.kP = 7.0;
    pidConfigs.kI = 0.0;
    pidConfigs.kD = 0.06;
    pidConfigs.kV = 12.00 / (100.0000 / GEAR_RATIO);
    pidConfigs.kA = 0.045;

    MotionMagicConfigs mm_configs = armConfig.MotionMagic;
    mm_configs.MotionMagicCruiseVelocity = AlgaeintakeConstants.mm_cruisevel;
    mm_configs.MotionMagicAcceleration = AlgaeintakeConstants.mm_accel;
    mm_configs.MotionMagicJerk = AlgaeintakeConstants.mm_jerk;

    FeedbackConfigs fdb_configs = armConfig.Feedback;

    fdb_configs.SensorToMechanismRatio = GEAR_RATIO;

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = arm.getConfigurator().apply(armConfig);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Real Error, Could not configure device. Error: " + status.toString());
    }

    status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = wheel.getConfigurator().apply(wheelConfig);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Real Error, Could not configure device. Error: " + status.toString());
    }

    BaseStatusSignal.waitForAll(
        250.0, armPosition, armVelocity, armAppliedVolts, armCurrent, armGoal);
    arm.optimizeBusUtilization();

    BaseStatusSignal.waitForAll(250.0, wheelVelocity, wheelAppliedVolts, wheelCurrent);
    wheel.optimizeBusUtilization();

    pidd.reset(0.0);
    arm.setPosition(0.0);
  }

  @Override
  public void updateInputs(AlgaeintakeIOInputs inputs) {

    BaseStatusSignal.refreshAll(armPosition, armVelocity, armAppliedVolts, armCurrent, armGoal);

    BaseStatusSignal.refreshAll(wheelVelocity, wheelAppliedVolts, wheelCurrent);

    inputs.positionRad = Units.rotationsToRadians(armPosition.getValueAsDouble());
    inputs.velocityRadPerSec = Units.rotationsToRadians(armVelocity.getValueAsDouble());
    inputs.appliedVolts = armAppliedVolts.getValueAsDouble();
    inputs.currentAmps =
        new double[] {armCurrent.getValueAsDouble(), wheelCurrent.getValueAsDouble()};
    SmartDashboard.putNumber("YOOOO Wheel Current", wheelCurrent.getValueAsDouble());
    SmartDashboard.putNumber("YOOOO Wheel Volts", wheelAppliedVolts.getValueAsDouble());
    inputs.goalPos = goalPos;
    inputs.goalVel = goalVel;

    inputs.wheelVelocityRadPerSec = Units.rotationsToRadians(wheelVelocity.getValueAsDouble());
    inputs.setpointPos = armGoal.getValueAsDouble();
    inputs.wheelAppliedVolts = wheelAppliedVolts.getValueAsDouble();
  }

  @Override
  public void setVoltage(double volts) {
    arm.setControl(new VoltageOut(volts));
  }

  @Override
  public void stop() {
    arm.stopMotor();
  }

  @Override
  public void runWheelVolts(double volts) {
    wheel.setControl(new VoltageOut(volts));
  }

  @Override
  public void stopWheels() {
    wheel.stopMotor();
  }

  @Override
  public void setPosition(double positionRad, double ffVolts) {
    goalPos = positionRad;
    arm.setControl(mm_volt.withPosition(Units.radiansToRotations(positionRad)).withSlot(0));
  }

  @Override
  public void resetPosition() {
    arm.setPosition(0);
  }

  public double getCurrent() {
    // return intakenPiece.calculate(wheel.getOutputCurrent());
    return wheelCurrent.getValueAsDouble();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    // pid.setP(kP);
    // pid.setI(kI);
    // pid.setD(kD);
    // pid.setFF(0);
    pidd.setPID(kP, kI, kD);
  }
}
