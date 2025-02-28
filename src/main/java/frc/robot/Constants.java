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

import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final class AlgaeIntakeConstants {
    public static final int pivotID = 23; // change
    public static final int wheelID = 55;

    public static final double GEAR_RATIO = 3.0 * 4.0; // check this one

    public static final double reefPickupA = 0.0; // lower
    public static final double reefPickupB = 0.0;
    public static final double groundPickup = 0.0;
    public static final double coralPickup = 0.0;
    public static final double stow = 0.0;

    public static final double intakeVolts = 8.0;
    public static final double holdVolts = 1.0;
    public static final double outtakeVolts = -6.0;

    public static final double tolerance = 0.05;
    public static final double downtolerance = 0.085;

    public static final double mm_cruisevel = 3.25;
    public static final double mm_accel = mm_cruisevel * 4.5;
    public static final double mm_jerk = mm_accel * 4.0;
  }

  public static final class HangConstants {
    public static final double liftUpVolts = -8.0;
    public static final double liftDownVolts = 8.0;
    public static final double holdVolts = -1.0;
    public static final double tolerance = 0.00;
    public static final double downtolerance = 0.00;
    public static final double dropDown = 0.00;
  }

  public static final class CoralIntakeConstants {
    public static final int pivotID = 100; // need to change
    public static final int wheelID = 101;

    public static final double GEAR_RATIO = 3.0 * 4.0;

    public static final double level1 = 0.0;
    public static final double level2 = 0.0;
    public static final double level3 = 0.0;
    public static final double level4 = 0.0;
    public static final double stationPickup = 0.0;
    public static final double stow = 0.0;

    public static final double intakeVolts = 8.0;
    public static final double holdVolts = 1.0;
    public static final double outtakeVolts = -6.0;

    public static final double tolerance = 0.05;

    public static final double mm_cruisevel = 3.25;
    public static final double mm_accel = mm_cruisevel * 4.5;
    public static final double mm_jerk = mm_accel * 4.0;
  }

  public static final class ElevatorConstants {
    public static final int leftElevatorID = 0; // check
    public static final int rightElevatorID = 61;

    public static final double GEAR_RATIO = 20.0;

    public static final double groundPos = 0.0;
    public static final double level1 = 0.0;
    public static final double level2 = 0.0;
    public static final double level3 = 0.0;
    public static final double level4 = 0.0;
    public static final double pickupA = 0.0;
    public static final double pickupB = 0.0;
    public static final double coralStation = 0.0;

    public static final double mm_cruisevel = 1.5;
    public static final double mm_accel = mm_cruisevel * 4.5;
    public static final double mm_jerk = mm_accel * 4.0;

    public static final double tolerance = 0.07;

    public static final double kTopLimit = 6.5; // radians
    public static final double kBottomLimit = 0.05; // ?
  }
}
