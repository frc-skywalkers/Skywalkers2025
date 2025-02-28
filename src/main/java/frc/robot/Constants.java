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

  public static final class AlgaeintakeConstants {
    public static final double intakeVolts = -8.0;
    public static final double holdVolts = -1.0;
    public static final double outtakeVolts = 6.0;

    public static final double mm_cruisevel = 3.25;
    public static final double mm_accel = mm_cruisevel * 4.5;
    public static final double mm_jerk = mm_accel * 4.0;

    public static final double dropDown = 3.025;
    public static final double tolerance = 0.05;
    public static final double downtolerance = 0.085;
  }

  public static final class HangConstants {
    public static final double liftUpVolts = -8.0;
    public static final double liftDownVolts = 8.0;
    public static final double holdVolts = -1.0;

    public static final double mm_cruisevel = 3.25;
    public static final double mm_accel = mm_cruisevel * 4.5;
    public static final double mm_jerk = mm_accel * 4.0;

    public static final double dropDown = 3.025;
    public static final double tolerance = 0.05;
    public static final double downtolerance = 0.085;
  }
}
