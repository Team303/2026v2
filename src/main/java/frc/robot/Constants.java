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
      public static double kP = 3;
      public static double kD = 0;
      public static double tolerance = 0.01; // ~.4 inches
    }

    public static class YController {
      public static double kP = 3;
      public static double kD = 0;
      public static double tolerance = 0.01;
    }

    public static class ThetaController {
      public static double kP = 30;
      public static double kD = 0;
      public static double tolerance = Units.degreesToRadians(1);
    }
  }

  public static class Spindexer{
    public static final int SPINDEXER_MOTOR_ID = 0; //change
    public static final int KICKER_MOTOR_ID = 0; //change
  }

  public static class Climber {
    public static final int CLIMBER_MOTOR_ID = 20;
  }

  public static class IntakeBelt {
    public static final int INTAKEBELT_MOTOR_ID = 0;
  }
}
