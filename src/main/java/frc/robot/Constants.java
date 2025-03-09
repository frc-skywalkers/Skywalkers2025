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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
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
    public static final int pivotID = 54; // change
    public static final int wheelID = 55;

    public static final double GEAR_RATIO = 45.0;

    public static final double outPos = -0.8;
    public static final double stow = 0.05;

    public static final double intakeVolts = -8.5;
    public static final double holdVolts = -1.0; // not used
    public static final double outtakeVolts = 4.0;

    public static final double tolerance = 0.1;
    public static final double downtolerance = 0.085;

    public static final double mm_cruisevel = 0.5; // 3.25
    public static final double mm_accel = mm_cruisevel * 4.5;
    public static final double mm_jerk = mm_accel * 4.0;
  }

  public static final class CoralIntakeConstants {
    public static final int pivotID = 40;
    public static final int wheelID = 51; // need to change

    public static final double GEAR_RATIO = 4.0 * 4.0 * 5.0;

    public static final double level1 = 1.445; // who knows bruh
    public static final double level2 = 1.617;
    public static final double level3 = 1.617;
    public static final double level4 = 0.0; // hahahahahahahahaha
    public static final double stationPickup = 0.485; // ? 0.518
    public static final double stow = 0.1;
    public static final double horiz = 0.767;

    public static final double intakeVolts = 4.0;
    public static final double holdVolts = 0.5;
    public static final double outtakeVolts = -2.0;

    public static final double tolerance = 0.05;

    public static final double mm_cruisevel = 0.6;
    public static final double mm_accel = mm_cruisevel * 4.5;
    public static final double mm_jerk = mm_accel * 4.0;

    public static final int encoderID = 300;
  }

  public static final class ElevatorConstants {
    public static final int leftElevatorID = 60; // check
    public static final int rightElevatorID = 61;

    public static final double GEAR_RATIO = 20.0;

    public static final double groundPos = 0.4;
    public static final double level1 = 6.035; // BAD CONSTANT FAKE CONSTANT DO NOT USE FR
    public static final double level2 = 6.148;
    public static final double level3 = 15.642;
    public static final double level4 = 0.0; // hahahah
    public static final double upperAlgae = 24.463;
    public static final double lowerAlgae = 14.674;
    public static final double coralStation = 3.188; // BAD!!

    public static final double mm_cruisevel = 3.5;
    public static final double mm_accel = mm_cruisevel * 4.5;
    public static final double mm_jerk = mm_accel * 4.0;

    public static final double tolerance = 0.02;

    public static final double kTopLimit = 25.0; // radians
    public static final double kBottomLimit = 0; // ?
    public static final int encoderID = 62; // change this number wheehoo
  }

  public static final class HangConstants {
    public static final double GEAR_RATIO = 225.0;
    public static final double liftUpVolts = -8.0;
    public static final double liftDownVolts = 8.0;
    public static final double holdVolts = -1.0;
    public static final double tolerance = 0.05;
    public static final double downtolerance = 0.00;
    public static final double dropDown = 0.00;

    public static final double mm_cruisevel = 0.2;
    public static final double mm_accel = mm_cruisevel * 10.0;
    public static final double mm_jerk = mm_accel * 10.0;

    public static final int hangID = 20;
    public static final int encoderID = 21;
  }

  public static final class VisionConstants {
    public static final double cameraHeight = Units.inchesToMeters(9.85);

    public static Pose2d[] leftReefPoses = {
      null,
      null,
      null,
      null,
      null,
      null,
      new Pose2d(
          13.686, 2.494, new Rotation2d(Units.degreesToRadians(120))), // IDs 6-11 on red reef
      new Pose2d(14.709, 3.848, new Rotation2d(Units.degreesToRadians(180))),
      new Pose2d(14.028, 5.325, new Rotation2d(Units.degreesToRadians(240))),
      new Pose2d(12.398, 5.525, new Rotation2d(Units.degreesToRadians(300))),
      new Pose2d(11.451, 4.171, new Rotation2d(Units.degreesToRadians(0))),
      new Pose2d(12.112, 2.698, new Rotation2d(Units.degreesToRadians(60))),
      null,
      null,
      null,
      null,
      null,
      new Pose2d(
          3.538, 2.704, new Rotation2d(Units.degreesToRadians(60))), // IDs 17-22 on blue reef
      new Pose2d(2.848, 4.187, new Rotation2d(Units.degreesToRadians(0))),
      new Pose2d(3.858, 5.540, new Rotation2d(Units.degreesToRadians(300))),
      new Pose2d(5.470, 5.325, new Rotation2d(Units.degreesToRadians(240))),
      new Pose2d(6.100, 3.862, new Rotation2d(Units.degreesToRadians(180))),
      new Pose2d(5.156, 2.564, new Rotation2d(Units.degreesToRadians(120))),
    };

    public static Pose2d[] rightReefPoses = {
      null,
      null,
      null,
      null,
      null,
      null,
      new Pose2d(
          14.025, 2.687, new Rotation2d(Units.degreesToRadians(120))), // IDs 6-11 on red reef
      new Pose2d(14.701, 4.201, new Rotation2d(Units.degreesToRadians(180))),
      new Pose2d(13.711, 5.545, new Rotation2d(Units.degreesToRadians(240))),
      new Pose2d(12.113, 5.358, new Rotation2d(Units.degreesToRadians(300))),
      new Pose2d(11.420, 3.858, new Rotation2d(Units.degreesToRadians(0))),
      new Pose2d(12.386, 2.527, new Rotation2d(Units.degreesToRadians(60))),
      null,
      null,
      null,
      null,
      null,
      new Pose2d(
          3.809, 2.520, new Rotation2d(Units.degreesToRadians(60))), // IDs 17-22 on blue reef
      new Pose2d(2.843, 3.867, new Rotation2d(Units.degreesToRadians(0))),
      new Pose2d(3.523, 5.351, new Rotation2d(Units.degreesToRadians(300))),
      new Pose2d(5.171, 5.520, new Rotation2d(Units.degreesToRadians(240))),
      new Pose2d(6.127, 4.190, new Rotation2d(Units.degreesToRadians(180))),
      new Pose2d(5.456, 2.690, new Rotation2d(Units.degreesToRadians(120))),
    };

    public static Pose2d[] otherPoses = {
      null,
      new Pose2d(
          16.089, 1.018, new Rotation2d(Units.degreesToRadians(305.818))), // ID 1 coral station
      new Pose2d(
          16.399, 7.036, new Rotation2d(Units.degreesToRadians(54.851))), // ID 2 coral station
      new Pose2d(11.540, 7.457, new Rotation2d(Units.degreesToRadians(90))), // ID 3 processor
      null,
      null,
      null,
      null,
      null,
      null,
      null,
      null,
      new Pose2d(
          1.309, 0.895, new Rotation2d(Units.degreesToRadians(233.991))), // ID 12 coral station
      new Pose2d(
          1.175, 7.031, new Rotation2d(Units.degreesToRadians(125.476))), // ID 13 coral station
      null,
      null,
      new Pose2d(5.969, 0.638, new Rotation2d(Units.degreesToRadians(270.0))) // ID 16 processor
    };
  }
}
