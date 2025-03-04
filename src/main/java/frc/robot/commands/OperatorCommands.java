package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.algaeintake.AlgaeIntake;
import frc.robot.subsystems.coralintake.CoralIntake;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.hang.Hang;

public class OperatorCommands {
  private OperatorCommands() {}

  // public static Command zeroElevator(Elevator elevator) {
  //   return Commands.run(
  //           () -> {
  //             elevator.runVolts(-4.0);
  //             elevator.disableSoftLimits();
  //           },
  //           elevator)
  //       .until(() -> elevator.isHomed())
  //       .andThen(
  //           () -> {
  //             elevator.resetPosition();
  //             elevator.stop();
  //             elevator.isZeroed = true;
  //             elevator.enableSoftLimits();
  //             System.out.println("DONE DONE DONE DONE DONE DONE");
  //           },
  //           elevator);
  // }

  // public static Command goToPosition(
  //     Elevator elevator, AlgaeIntake a_intake, double e_pos, double a_pos) {
  //   return Commands.run(
  //           () -> {
  //             elevator.goToPosition(e_pos);
  //             a_intake.goToPosition(a_pos);
  //           },
  //           elevator,
  //           a_intake)
  //       .until(() -> (a_intake.atPosition(a_pos)) && (elevator.atPosition(e_pos)));
  // }

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
}
