package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
// import frc.robot.subsystems.algaeintake.AlgaeIntake;
import frc.robot.subsystems.elevator.Elevator;

public class OperatorCommands {
  private OperatorCommands() {}

  public static Command zeroElevator(Elevator elevator) {
    return Commands.run(
            () -> {
              elevator.runVolts(-2.0);
            },
            elevator)
        .until(() -> elevator.isHomed())
        .andThen(
            () -> {
              elevator.resetPosition();
              elevator.stop();
            },
            elevator);
  }

  //   public static Command goToPosition(
  //       Elevator elevator, AlgaeIntake a_intake, double e_pos, double a_pos) {
  //     return Commands.run(
  //             () -> {
  //               elevator.goToPosition(e_pos);
  //               a_intake.goToPosition(a_pos);
  //             },
  //             elevator,
  //             a_intake)
  //         .until(() -> (a_intake.atPosition(a_pos)) && (elevator.atPosition(e_pos)));
  //   }
}
