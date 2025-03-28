// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.FeedForwardCharacterization;
import frc.robot.commands.OperatorCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.coralintake.CoralIntake;
import frc.robot.subsystems.coralintake.CoralIntakeIO;
import frc.robot.subsystems.coralintake.CoralIntakeIOTalonFX;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  //   private final AlgaeIntake a_intake;
  private final CoralIntake c_intake;
  private final Elevator elevator;
  //   private final Hang hang;
  //   private final Vision vision;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);
  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        // a_intake = new AlgaeIntake(new AlgaeIntakeIOTalonFX());
        c_intake = new CoralIntake(new CoralIntakeIOTalonFX());
        // hang = new Hang(new HangIOTalonFX());
        elevator = new Elevator(new ElevatorIOTalonFX());

        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        // a_intake = new AlgaeIntake(new AlgaeIntakeIO() {});
        c_intake = new CoralIntake(new CoralIntakeIO() {});
        // hang = new Hang(new HangIO() {});
        elevator = new Elevator(new ElevatorIO() {});
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        // a_intake = new AlgaeIntake(new AlgaeIntakeIO() {});
        c_intake = new CoralIntake(new CoralIntakeIO() {});
        // hang = new Hang(new HangIO() {});
        elevator = new Elevator(new ElevatorIO() {});
        break;
    }

    NamedCommands.registerCommand("elevator up", OperatorCommands.moveElevator(elevator, 6.63));

    // NamedCommands.registerCommand("elevator down", OperatorCommands.moveElevator(elevator, 0));

    // NamedCommands.registerCommand(
    //     "algae pivot out",
    //     OperatorCommands.moveAlgae(a_intake, AlgaeIntakeConstants.stow)); // check value

    // NamedCommands.registerCommand(
    //     "coral pivot down",
    //     OperatorCommands.moveCoral(c_intake, CoralIntakeConstants.horiz)); // horizontal value
    // 0.767 is the horizontal value

    // NamedCommands.registerCommand("L1", OperatorCommands.goToLevel1(c_intake, elevator,
    // a_intake));
    // NamedCommands.registerCommand("L2", OperatorCommands.goToLevel2(c_intake, elevator,
    // a_intake));
    // NamedCommands.registerCommand("L3", OperatorCommands.goToLevel3(c_intake, elevator,
    // a_intake));
    // NamedCommands.registerCommand(
    //     "ground", OperatorCommands.goToGround(c_intake, elevator, a_intake));
    // NamedCommands.registerCommand("outtakeCoral", OperatorCommands.outtakeCoral(c_intake));
    // NamedCommands.registerCommand("intakeCoral", OperatorCommands.intakeCoral(c_intake));
    // NamedCommands.registerCommand(
    //     "coralStation", OperatorCommands.goToStation(c_intake, elevator, a_intake));

    // NamedCommands.registerCommand("L3 outtake", OperatorCommands.outtakeL3(elevator, c_intake));

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // autoChooser.addOption(
    //     "algae characterization",
    //     new FeedForwardCharacterization(
    //         a_intake, a_intake::runVolts, a_intake::getCharacterizationVelocity));

    autoChooser.addOption(
        "elevator characterization",
        new FeedForwardCharacterization(
            elevator, elevator::runVolts, elevator::getCharacterizationVelocity));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive

    if (controller.rightTrigger().getAsBoolean()) {
      drive.setDefaultCommand(
          DriveCommands.joystickDrive(
              drive,
              () -> -controller.getLeftY() * 0.4,
              () -> -controller.getLeftX() * 0.4,
              () -> -controller.getRightX() * 0.4));
    } else {
      drive.setDefaultCommand(
          DriveCommands.joystickDrive(
              drive,
              () -> -controller.getLeftY(),
              () -> -controller.getLeftX(),
              () -> -controller.getRightX()));
    }

    // joystick hang for testing
    // hang.setDefaultCommand(
    //     Commands.run(
    //         () -> {
    //           hang.runVolts(operator.getLeftY());
    //         },
    //         hang));

    // // Lock to 0° when A button is held
    // controller
    //     .a()
    //     .whileTrue(
    //         DriveCommands.joystickDriveAtAngle(
    //             drive,
    //             () -> -controller.getLeftY(),
    //             () -> -controller.getLeftX(),
    //             () -> new Rotation2d()));

    // // Switch to X pattern when X button is pressed
    // controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed //CHANGE FOR COMP, 180
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d(0))),
                    drive)
                .ignoringDisable(true));

    // set gyro to latest approved vision measurement

    // CCR
    operator.a().onTrue(OperatorCommands.moveElevator(elevator, ElevatorConstants.level1));
    operator.x().onTrue(OperatorCommands.moveElevator(elevator, ElevatorConstants.level2));
    operator.b().onTrue(OperatorCommands.moveElevator(elevator, ElevatorConstants.level3));
    operator.y().onTrue(OperatorCommands.moveElevator(elevator, ElevatorConstants.level4));

    operator.leftBumper().onTrue(OperatorCommands.intakeCoral(c_intake));
    operator.leftTrigger().onTrue((OperatorCommands.outtakeCoral(c_intake)));

    operator.povUp().onTrue(OperatorCommands.moveElevator(elevator, ElevatorConstants.coralStation));
    operator.povDown().onTrue(OperatorCommands.moveElevator(elevator, ElevatorConstants.groundPos));

    //dangerous
    controller.leftBumper().onTrue(DriveCommands.alignReef(drive, controller));


    // operator.a().onTrue(OperatorCommands.moveElevator(elevator, 9.0)); // L2
    // operator.b().onTrue(OperatorCommands.moveElevator(elevator, 17.0)); // L3
    // operator.y().onTrue(OperatorCommands.moveElevator(elevator, 24.5));
    // operator.x().onTrue(OperatorCommands.moveElevator(elevator, 0.0));


    // operator.y().onTrue(DriveCommands.alignReef(drive, operator));

    // VCR CONTROLS
    // operator.povDown().onTrue(OperatorCommands.goToLowerAlgae(c_intake, elevator, a_intake));
    // operator.povUp().onTrue(OperatorCommands.goToUpperAlgae(c_intake, elevator, a_intake));
    // operator.povLeft().onTrue(OperatorCommands.goToGround(c_intake, elevator, a_intake));
    // operator.povRight().onTrue(OperatorCommands.stowAll(c_intake, elevator, a_intake));
    // operator.leftBumper().onTrue(OperatorCommands.outtakeAlgae(a_intake));
    // operator.leftTrigger().whileTrue(OperatorCommands.intakeAlgae(a_intake)); // check
    // operator.a().onTrue(OperatorCommands.goToLevel1(c_intake, elevator, a_intake));
    // operator.x().onTrue(OperatorCommands.goToLevel2(c_intake, elevator, a_intake));
    // operator.y().onTrue(OperatorCommands.goToLevel3(c_intake, elevator, a_intake));
    // operator.b().onTrue(OperatorCommands.goToStation(c_intake, elevator, a_intake));
    // operator.rightBumper().onTrue(OperatorCommands.outtakeCoral(c_intake));
    // operator.rightTrigger().whileTrue(OperatorCommands.intakeCoral(c_intake)); // check

    // *** TESTING *** ///
    // elevator.setDefaultCommand(
    //     Commands.runOnce(() -> elevator.runVolts(operator.getLeftX() * 3.8), elevator));
    // c_intake.setDefaultCommand(
    //     Commands.runOnce(() -> c_intake.runVolts(operator.getRightX() * 1.2), c_intake));

    // operator.x().onTrue(Commands.runOnce(() -> c_intake.runWheelVolts(4.0)));
    // operator.y().onTrue(Commands.runOnce(() -> c_intake.runWheelVolts(-0.5)));
    // operator.leftBumper().onTrue(Commands.runOnce(() -> c_intake.stopWheels()));

    // operator.a().onTrue(Commands.runOnce(() -> a_intake.runWheelVolts(4.0)));
    // operator.b().onTrue(Commands.runOnce(() -> a_intake.runWheelVolts(-4.0)));
    // operator.rightBumper().onTrue(Commands.runOnce(() -> a_intake.stopWheels()));

    // operator.leftTrigger().onTrue(Commands.runOnce(() -> elevator.goToPosition(6.63)));
    // operator.rightTrigger().onTrue(Commands.runOnce(() -> c_intake.goToPosition(0.5)));

    // operator.a().onTrue(OperatorCommands.coralPickup(elevator, c_intake));
    // operator.b().onTrue(OperatorCommands.outtakeL1(elevator, c_intake));

    // operator.rightBumper().onTrue(OperatorCommands.coralPickup(elevator, c_intake));

    // operator.b().onTrue(OperatorCommands.testAndThen(elevator, c_intake, 8.0, 0.5));

    // operator.b().onTrue(OperatorCommands.intakeCoral(c_intake));

    // PIDController aimController = new PIDController(0.2, 0.0, 0.0);
    // aimController.enableContinuousInput(-Math.PI, Math.PI);
    // controller
    //     .y()
    //     .whileTrue(
    //         Commands.startRun(
    //             () -> {
    //               aimController.reset();
    //             },
    //             () -> {
    //               drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0,
    // vision.getTargetX(0).getRadians(), drive.getRotation()));
    //             },
    //             drive));

    // controller
    //     .leftBumper()
    //     .whileTrue(
    //         DriveCommands.autoAlign(drive, controller, vision.getPrimaryTargetPose(0))
    //     );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
