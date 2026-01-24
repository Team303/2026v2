// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.util.LoggedTunableNumber;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final boolean tuningMode = true;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;
  public static final String CAN_BUS_TOP_NAME = "NOT Drivebase";

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static class DriveToPoseConstraints {
    public static double maxVelocityMPS = 1000;
    public static double maxAccelerationMPSSq = 30;
    public static double maxAngularVelocityRadPerSec = 30;
    public static double maxAngularAccelerationRadPerSecSq = 12;

    public static LoggedTunableNumber maxVelocityMPSScaler =
        new LoggedTunableNumber("DriveToPoseConstaints/maxVelocityMPSScaler", 1);
    public static LoggedTunableNumber maxAccelerationMPSSqScaler =
        new LoggedTunableNumber("DriveToPoseConstaints/maxAccelerationMPSSqScaler", 1);
    public static LoggedTunableNumber maxAngularVelocityRadPerSecScaler =
        new LoggedTunableNumber("DriveToPoseConstaints/maxAngularVelocityRadPerSecScaler", 1);
    public static LoggedTunableNumber maxAngularAccelerationRadPerSecSqScaler =
        new LoggedTunableNumber("DriveToPoseConstaints/maxAngularAccelerationRadPerSecSqScaler", 1);

    public static PathConstraints fastpathConstraints =
        new PathConstraints(
            maxVelocityMPS,
            maxAccelerationMPSSq,
            maxAngularVelocityRadPerSec,
            maxAngularAccelerationRadPerSecSq);

    public static PathConstraints slowpathConstraints =
        new PathConstraints(
            maxVelocityMPS * maxVelocityMPSScaler.get(),
            maxAccelerationMPSSq * maxAccelerationMPSSqScaler.get(),
            maxAngularVelocityRadPerSec * maxAngularVelocityRadPerSecScaler.get(),
            maxAngularAccelerationRadPerSecSq * maxAngularAccelerationRadPerSecSqScaler.get());

    public static void updateTunableNumbers() {
      slowpathConstraints =
          new PathConstraints(
              maxVelocityMPS * maxVelocityMPSScaler.get(),
              maxAccelerationMPSSq * maxAccelerationMPSSqScaler.get(),
              maxAngularVelocityRadPerSec * maxAngularVelocityRadPerSecScaler.get(),
              maxAngularAccelerationRadPerSecSq * maxAngularAccelerationRadPerSecSqScaler.get());
    }
  }

  public static class DriveToPoseStraight {

    public static double maxVelocityMPS = 1000;
    public static double maxAngularVelocityRadPerSec = 300;

    public static class XController {
      public static double kP = 9;
      public static double kD = 0;
      public static double tolerance = 0.01; // ~.4 inches
    }

    public static class YController {
      public static double kP = 4.5;
      public static double kD = 0;
      public static double tolerance = 0.01;
    }

    public static class ThetaController {
      public static double kP = 7;
      public static double kD = 0;
      public static double tolerance = Units.degreesToRadians(0.25);
    }
  }

  public static class Shooter {
    // TURRET CONSTANTS
    public static class Turret {
      public static final int TURRET_THROUGHBORE_ID = 14; // NEED TO CHANGE
      public static final int TURRET_MOTOR_ID = 41; // NEED TO CHANGE
      public static final double TURRET_kS = 0.4;
      public static final double TURRET_kP = 6;
      public static final double TURRET_kI = 0.0;
      public static final double TURRET_kD = 0.05;
      public static final double TURRET_kA = 0.0;

      public static final double TURRET_maxV = 100;
      public static final double TURRET_maxA = 200;

      public static final double TURRET_MOTOR_THROUGHBORE_RATIO = 2.8;

      public static final double TURRET_HOME_POS = -0.153564453125;
      public static final double TEST_HUB_POS = 0.139404296875;
    }
    // FLYWHEEL CONSTANTS --> Need to tune all PID 
    public static class Flywheel {
      public static final int FLYWHEEL_LEFT_MOTOR_ID = 99; // NEED TO CHANGE
      public static final int FLYWHEEL_RIGHT_MOTOR_ID = 99; // NEED TO CHANGE
      public static final double FLYWHEEL_kS = 0.0;
      public static final double FLYWHEEL_kP = 0.0;
      public static final double FLYWHEEL_kI = 0.0;
      public static final double FLYWHEEL_kD = 0.0;
      public static final double FLYWHEEL_kV = 0.0;

      public static final double FLYWHEEL_maxV = 10;
      public static final double FLYWHEEL_maxA = 20;
    }
    // HOOD CONSTANTS --> NEED TO TUNE ALL PID
    public static class Hood {
      public static final int HOOD_THROUGHBORE_ID = 99; // NEED TO CHANGE
      public static final int HOOD_MOTOR_ID = 99; // NEED TO CHANGE
      public static final double HOOD_kS = 0.0;
      public static final double HOOD_kP = 0.0;
      public static final double HOOD_kI = 0.0;
      public static final double HOOD_kD = 0.0;

      public static final double HOOD_maxV = 10;
      public static final double HOOD_maxA = 20;

      public static final double HOOD_HOME_POS = 0.0; // NEED TO CHANGE
    }
  }

  public static class Algae {
    public static final int INTAKE_MOTOR_ID = 32;
    public static final int PIVOT_MOTOR_ID = 31;
    public static final double PIVOT_MOTOR_Kp = 0.1;
    public static final double PIVOT_FEED_FORWARD_KS = 0;
    public static final double PIVOT_FEED_FORWARD_KG = 0.22;
    public static final double PIVOT_FEED_FORWARD_KV = 1.98;
    public static final double PIVOT_FEED_FORWARD_KA = 0.01;

    public static final double HOME_POSITION = -0.445;
    public static final double DEALGAE_POSITION = -0.230;
  }

  public static class EndEffector {
    public static final int LEFT_END_EFFECTOR_MOTOR_ID = 22; // Need to change
    public static final int RIGHT_END_EFFECTOR_MOTOR_ID = 23; // Need to change
    public static final int FIRST_LC_ID = 0;
    public static final int SECOND_LC_ID = 1;
  }

  public static class Elevator {
    public static final int LEFT_ELEVATOR_MOTOR_ID = 24;
    public static final int RIGHT_ELEVATOR_MOTOR_ID = 25;

    public static final double ELEVATOR_ENCODER_OFFSET = 9.08837890625;
  }

  public static class Indexer {
    public static final int INDEXER_MOTOR_ID = 37; // Need to change
  }

  public static class EndEffectorV2Subsystem {
    public static final int SWIVEL_END_EFFECTOR_MOTOR_ID = 31;
    public static final int BELT_END_EFFECTOR_MOTOR_ID = 32;
    public static final int CAN_RANGE_ID = 50;
    public static final int CAN_CODER_ID = 41;
    public static final double HOME_POS = 0.04;
    public static final double SHOOT_POS = 0.54; // tune
    public static final double THRESHOLD = 0; // tune
  }

  public static class Intake {
    public static final int PIVOT_INTAKE_MOTOR_ID = 30;
    public static final int BELT_INTAKE_MOTOR_ID = 34;
    public static final int INTAKE_CAN_CODER_ID = 40;
    public static final double HOME_POS = 0.76;
    public static final double DOWN_POS = 0.1; // tune
  }
}
