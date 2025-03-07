package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.CoralIntakeConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.algaeintake.AlgaeIntake;
import frc.robot.subsystems.coralintake.CoralIntake;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.hang.Hang;

public class OperatorCommands {
  private OperatorCommands() {}

  public static Command zeroElevator(Elevator elevator) {
    return Commands.run(
            () -> {
              elevator.runVolts(-4.0); // was -2
            },
            elevator)
        .until(() -> elevator.isHomed())
        .andThen(
            () -> {
              elevator.resetPosition();
              elevator.stop();
              System.out.println("DONE DONE DONE DONE DONE DONE");
            },
            elevator);
  }

  // public static Command goToPreset(
  //     Elevator elevator, CoralIntake c_intake, double e_pos, double c_pos) {
  //   return Commands.run(
  //           () -> {
  //             elevator.goToPosition(e_pos);
  //             c_intake.goToPosition(c_pos);
  //           },
  //           elevator,
  //           c_intake)
  //       .until(() -> (c_intake.atPosition(c_pos)) && (elevator.atPosition(e_pos)));
  // }

  public static Command goToPreset(
      Elevator elevator, AlgaeIntake a_intake, double e_pos, double a_pos) {
    return Commands.run(
            () -> {
              elevator.goToPosition(e_pos);
              a_intake.goToPosition(a_pos);
            },
            elevator,
            a_intake)
        .until(() -> (a_intake.atPosition(a_pos)) && (elevator.atPosition(e_pos)));
  }

  // public static Command testAndThen(Elevator elevator, CoralIntake c_intake, double e_pos, double
  // c_pos) {
  //   return Commands.run(() -> {c_intake.goToPosition(c_pos);}, c_intake).andThen(() ->
  // {elevator.goToPosition(e_pos);}, elevator);
  // }

  public static Command testAndThen(
      Elevator elevator, AlgaeIntake a_intake, double e_pos, double a_pos) {
    return Commands.run(
            () -> {
              a_intake.goToPosition(a_pos);
            },
            a_intake)
        .withTimeout(1.5)
        .andThen(
            () -> {
              elevator.goToPosition(e_pos);
            },
            elevator)
        .until(() -> (a_intake.atPosition(a_pos) && elevator.atPosition(e_pos)));

    // .until(() -> elevator.atPosition(e_pos))
    // .andThen(
    //     () -> {
    //       elevator.stop();
    //       a_intake.stop();
    //     },
    //     elevator,
    //     a_intake);
  }

  public static Command moveElevator(Elevator elevator, double e_pos) {
    return Commands.run(
            () -> {
              elevator.goToPosition(e_pos);
              System.out.println("went to position" + e_pos);
            },
            elevator)
        .until(() -> elevator.atPosition(e_pos));
  }

  public static Command moveHang(Hang hang, double h_pos) {
    return Commands.run(
            () -> {
              hang.goToPosition(h_pos);
            },
            hang)
        .until(() -> hang.atPosition(h_pos));
  }

  public static Command moveAlgae(AlgaeIntake algae, double a_pos) {
    return Commands.run(
            () -> {
              algae.goToPosition(a_pos);
            },
            algae)
        .until(() -> algae.atPosition(a_pos));
  }

  public static Command moveCoral(CoralIntake coral, double c_pos) {
    return Commands.run(
            () -> {
              coral.goToPosition(c_pos);
            },
            coral)
        .until(() -> coral.atPosition(c_pos));
  }

  // fr commands to run during the game

  public static Command outtakeL1(
      Elevator elevator, CoralIntake coral, Drive drive, CommandXboxController joystick) {
    return DriveCommands.alignOther(drive, joystick) //need center alignment for L1
        .andThen(
            () -> {
              elevator.goToPosition(ElevatorConstants.level1);
              coral.goToPosition(CoralIntakeConstants.level1);
            },
            elevator,
            coral)
        .until(
            () ->
                (coral.atPosition(CoralIntakeConstants.level1)
                    && elevator.atPosition(ElevatorConstants.level1)));
  }

  public static Command outtakeL2(
      Elevator elevator, CoralIntake coral, Drive drive, CommandXboxController joystick) {
    return DriveCommands.alignReef(drive, joystick)
        .andThen(
            () -> {
              elevator.goToPosition(ElevatorConstants.level2);
              coral.goToPosition(CoralIntakeConstants.level2);
            },
            elevator,
            coral)
        .until(
            () ->
                (coral.atPosition(CoralIntakeConstants.level2)
                    && elevator.atPosition(ElevatorConstants.level2)));
  }

  public static Command outtakeL3(
      Elevator elevator, CoralIntake coral, Drive drive, CommandXboxController joystick) {
    return DriveCommands.alignReef(drive, joystick)
        .andThen(
            () -> {
              elevator.goToPosition(ElevatorConstants.level3);
              coral.goToPosition(CoralIntakeConstants.level3);
            },
            elevator,
            coral)
        .until(
            () ->
                (coral.atPosition(CoralIntakeConstants.level3)
                    && elevator.atPosition(ElevatorConstants.level3)));
  }

  public static Command outtakeL4(
      Elevator elevator, CoralIntake coral, Drive drive, CommandXboxController joystick) {
    return DriveCommands.alignReef(drive, joystick)
        .andThen(
            () -> {
              elevator.goToPosition(ElevatorConstants.level4);
              coral.goToPosition(CoralIntakeConstants.level4);
            },
            elevator,
            coral)
        .until(
            () ->
                (coral.atPosition(CoralIntakeConstants.level4)
                    && elevator.atPosition(ElevatorConstants.level4)));
  }

}
