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
import frc.robot.lightstrip.LedState;
import frc.robot.lightstrip.Range;
import frc.robot.lightstrip.TempLedState;

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

  public static final class LightstripConstants {
    public static int pwmPort = 9;
    public static int ledCount = 150;

    public static final class Ranges {
      public static Range intake = new Range(0, 19);
      public static Range drivetrain = new Range(20, 99);
      public static Range superstructure = new Range(100, 149);
      public static Range shooter = new Range(150, 199);
      public static Range full = new Range(0, ledCount);
    }

    public static LedState defaultState = new LedState(50, 0, 0, "Fade");
    public static LedState holdingState = new LedState(0, 0, 55, "Fade");
    public static TempLedState successSignal = new TempLedState(0, 55, 0, "Fast Blink", 1);
    public static TempLedState intake = new TempLedState(255, 255, 0, "Solid", 10);
  }
}
