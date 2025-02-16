package frc.robot.subsystems.Algaeintake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class AlgaeintakeIOSim implements AlgaeintakeIO {

  private final DCMotorSim armSim;
  private final DCMotorSim wheelSim;

  private static final double ARM_KP = 0.05;
  private static final double ARM_KD = 0.0;
  private static final double ARM_KS = 0.0;
  private final double ARM_MOTOR_INERTIA = 0.004;
  final double ARM_MOTOR_GEAR_RATIO = 2.0;
  private final double WHEEL_MOTOR_INERTIA = 0.004;
  final double WHEEL_MOTOR_GEAR_RATIO = 2.0;
  // private static final double Arm_KV_ROT = 0.91035; // Same units as TunerConstants: (volt *
  // secs) / rotation
  // private static final double Arm_KV = 0.0 / Units.rotationsToRadians(1.0 / DRIVE_KV_ROT);
  private static final double WHEEL_KP = 8.0;
  private static final double WHEEL_KD = 0.0;
  private static final DCMotor ARM_GEARBOX = DCMotor.getKrakenX60Foc(1);
  private static final DCMotor WHEEL_GEARBOX = DCMotor.getKrakenX60Foc(1);

  private boolean closedLoop = false;
  private boolean wheelClosedLoop = false;
  private double ffVolts = 0.0;
  private double appliedVolts = 0.0;
  private double goalPos = 0.00;
  private double setpointPos = 0.00;
  private double wheelAppliedVolts = 0.0;

  private PIDController armController = new PIDController(ARM_KP, 0, ARM_KD);
  private PIDController wheelController = new PIDController(WHEEL_KP, 0, WHEEL_KD);

  public AlgaeintakeIOSim() {
    armSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                ARM_GEARBOX, ARM_MOTOR_INERTIA, ARM_MOTOR_GEAR_RATIO),
            ARM_GEARBOX);
    wheelSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                WHEEL_GEARBOX, WHEEL_MOTOR_INERTIA, WHEEL_MOTOR_GEAR_RATIO),
            WHEEL_GEARBOX);
    wheelController.enableContinuousInput(WHEEL_KP, ARM_KD);
  }
  // Arm motor voltage needs to be changed
  @Override
  public void updateInputs(AlgaeintakeIOInputs inputs) {

    if (closedLoop) {
      appliedVolts = ffVolts + armController.calculate(armSim.getAngularVelocityRadPerSec());
    } else {
      armController.reset();
    }
    if (wheelClosedLoop) {
      wheelAppliedVolts = wheelController.calculate(wheelSim.getAngularPositionRad());
    } else {
      wheelController.reset();
    }

    double armMotorVoltage = 12.0; // Example voltage to the intake motor

    // Apply the voltage to the intake motor simulation
    armSim.setInput(armMotorVoltage);

    // Simulate the arm motor's voltage input (for the intake arm)
    double wheelMotorVoltage = 8.0; // Example voltage to the arm motor
    wheelSim.setInput(wheelMotorVoltage);
  }
}
