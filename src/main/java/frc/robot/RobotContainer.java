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
import frc.robot.commands.DriveCommands;
import frc.robot.commands.FeedForwardCharacterization;
import frc.robot.commands.OperatorCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.algaeintake.AlgaeIntake;
import frc.robot.subsystems.algaeintake.AlgaeIntakeIO;
import frc.robot.subsystems.algaeintake.AlgaeIntakeIOTalonFX;
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
import frc.robot.subsystems.hang.Hang;
import frc.robot.subsystems.hang.HangIO;
import frc.robot.subsystems.hang.HangIOTalonFX;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final AlgaeIntake a_intake;
  private final CoralIntake c_intake;
  private final Elevator elevator;
  private final Hang hang;
  //   private final Vision vision;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  private final LoggedNetworkNumber elevator_Pos =
      new LoggedNetworkNumber("/SmartDashboard/ePos", 6.63);

  public double multiplier = 1;

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
        a_intake = new AlgaeIntake(new AlgaeIntakeIOTalonFX());
        c_intake = new CoralIntake(new CoralIntakeIOTalonFX());
        hang = new Hang(new HangIOTalonFX());
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
        a_intake = new AlgaeIntake(new AlgaeIntakeIO() {});
        c_intake = new CoralIntake(new CoralIntakeIO() {});
        hang = new Hang(new HangIO() {});
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
        a_intake = new AlgaeIntake(new AlgaeIntakeIO() {});
        c_intake = new CoralIntake(new CoralIntakeIO() {});
        hang = new Hang(new HangIO() {});
        elevator = new Elevator(new ElevatorIO() {});
        break;
    }

    NamedCommands.registerCommand("elevator up", OperatorCommands.moveElevator(elevator, 6.63));

    NamedCommands.registerCommand("elevator down", OperatorCommands.moveElevator(elevator, 0));

    NamedCommands.registerCommand(
        "algae pivot out", OperatorCommands.moveAlgae(a_intake, -1.0)); // check value

    NamedCommands.registerCommand(
        "coral pivot down", OperatorCommands.moveCoral(c_intake, 0.0)); // check value

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
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY() * 0.25,
            () -> -controller.getLeftX() * 0.25,
            () -> -controller.getRightX() * 0.25));

    // joystick hang for testing
    // hang.setDefaultCommand(
    //     Commands.run(
    //         () -> {
    //           hang.runVolts(operator.getLeftY());
    //         },
    //         hang));

    // Lock to 0° when A button is held
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> new Rotation2d()));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    if (controller.rightBumper().getAsBoolean()) {
      multiplier = 0.25;
    }

    // controller.y().onTrue(Commands.runOnce(() -> a_intake.runVolts(4.0)));

    // operator.x().onTrue(Commands.runOnce(() -> elevator.goToPosition(elevator_Pos.get())));
    // System.out.println(elevator_Pos.get());

    // operator.x().onTrue(Commands.runOnce(() -> elevator.goToPosition(22.0)));
    // operator.y().onTrue(Commands.runOnce(() -> elevator.goToPosition(15.0)));
    // operator.a().onTrue(Commands.runOnce(() -> elevator.goToPosition(8.0)));

    // operator.b().onTrue(Commands.runOnce(() -> hang.goToPosition(-21.0)));

    operator.x().onTrue(Commands.runOnce(() -> c_intake.goToPosition(0.200)));
    operator.y().onTrue(Commands.runOnce(() -> c_intake.goToPosition(0.767)));

    // operator.y().onTrue(Commands.runOnce(() -> elevator.resetPosition()));

    // operator.a().onTrue(OperatorCommands.zeroElevator(elevator));

    operator.a().onTrue(OperatorCommands.testAndThen(elevator, c_intake, 22.0, 0.767));
    // operator.b().onTrue(OperatorCommands.testAndThen(elevator, c_intake, 8.0, 0.5));

    operator.b().onTrue(OperatorCommands.intakeCoral(c_intake));

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
