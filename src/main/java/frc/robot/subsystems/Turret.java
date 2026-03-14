package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.subsystems.drive.Drive;

import static frc.robot.RobotContainer.drive;
// import static frc.robot.RobotContainer.tempDrivebase;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Turret extends SubsystemBase {

  private final TalonFX turretMotor;
  private final CANcoder throughBore; // Powered by CANCoder

  private final LoggedNetworkNumber motorPosition;
  private final LoggedNetworkNumber throughBorePosition;

  public static LoggedTunableNumber TESTING_kP =
      new LoggedTunableNumber("TURRET TESTING_kP", Constants.Shooter.Turret.TURRET_kP);
  public static LoggedTunableNumber TESTING_kI =
      new LoggedTunableNumber("TURRET TESTING_kI", Constants.Shooter.Turret.TURRET_kI);
  public static LoggedTunableNumber TESTING_kD =
      new LoggedTunableNumber("TURRET TESTING_kD", Constants.Shooter.Turret.TURRET_kD);
  public static LoggedTunableNumber TESTING_kA =
      new LoggedTunableNumber("TURRET TESTING_kA", Constants.Shooter.Turret.TURRET_kA);
  public static LoggedTunableNumber TESTING_mmV =
      new LoggedTunableNumber("TURRET TESTING_mmV", Constants.Shooter.Turret.TURRET_maxV);
  public static LoggedTunableNumber TESTING_mmA =
      new LoggedTunableNumber("TURRET TESTING_mmA", Constants.Shooter.Turret.TURRET_maxA);

  public static LoggedTunableNumber GOAL_POS = new LoggedTunableNumber("TURRET GOAL_POS", Constants.Shooter.Turret.TEST_HUB_POS);

  private static int wallahi;

  // Field-relative velocity is sourced directly from the Pigeon-backed drive chassis speeds

  // Cached shoot-on-the-move values, refreshed every periodic() cycle
  private Translation2d cachedVirtualTarget = null;
  private double cachedVirtualTargetDistance = 0.0;
  private double cachedVelocityTowardHub = 0.0;

  private static final double BLUE_HUB_X = 4.6269;
  private static final double BLUE_HUB_Y = 4.03;
  private static final double RED_HUB_X = 11.91358;
  private static final double RED_HUB_Y = 4.03;

  public Turret() {
    throughBore = new CANcoder(Constants.Shooter.Turret.TURRET_THROUGHBORE_ID, "topside");
    CANcoderConfiguration cc_cfg = new CANcoderConfiguration();
    cc_cfg.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
    cc_cfg.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    cc_cfg.MagnetSensor.MagnetOffset = -0.5415625;
    throughBore.getConfigurator().apply(cc_cfg);

    turretMotor = new TalonFX(Constants.Shooter.Turret.TURRET_MOTOR_ID, "topside");

    turretMotor.setPosition(throughBore.getAbsolutePosition().getValueAsDouble());

    applyMainConfigs();

    throughBorePosition = new LoggedNetworkNumber("Turret Absolute Position", 0.0);
    motorPosition = new LoggedNetworkNumber("Turret Motor Position", 0.0);
    
    wallahi = 0;
  }

  public void printRAWPositionForOffset() {
    CANcoderConfiguration tempConfiguration = new CANcoderConfiguration();
    tempConfiguration.MagnetSensor.MagnetOffset = 0;
    tempConfiguration.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0;
    throughBore.getConfigurator().apply(tempConfiguration);

    throughBore.getAbsolutePosition().waitForUpdate(0.1);
    double rawPosition = throughBore.getAbsolutePosition().getValueAsDouble();

    System.out.println(rawPosition);
  }

  private void applyMainConfigs() {
    var turretMotorConfig = new TalonFXConfiguration();

    turretMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    turretMotorConfig.Feedback.FeedbackRemoteSensorID = throughBore.getDeviceID();
    turretMotorConfig.Feedback.SensorToMechanismRatio = 8.5;
    //turretMotorConfig.Feedback.RotorToSensorRatio = 8.5;

    var Slot0Configs = turretMotorConfig.Slot0;
    Slot0Configs.kS = Constants.Shooter.Turret.TURRET_kS;
    Slot0Configs.kP = Constants.Shooter.Turret.TURRET_kP;
    Slot0Configs.kI = Constants.Shooter.Turret.TURRET_kI;
    Slot0Configs.kD = Constants.Shooter.Turret.TURRET_kD;
    Slot0Configs.kA = Constants.Shooter.Turret.TURRET_kA;

    var motionMagicConfigs = turretMotorConfig.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = Constants.Shooter.Turret.TURRET_maxV;
    motionMagicConfigs.MotionMagicAcceleration = Constants.Shooter.Turret.TURRET_maxA;
    turretMotor.getConfigurator().apply(turretMotorConfig);

    var motorTalonFXConfigurator = turretMotor.getConfigurator();

    var limitConfigs = new CurrentLimitsConfigs();
    limitConfigs.StatorCurrentLimit = 60;
    limitConfigs.SupplyCurrentLimit = 60;
    limitConfigs.StatorCurrentLimitEnable = true;
    limitConfigs.SupplyCurrentLimitEnable = true;
    motorTalonFXConfigurator.apply(limitConfigs);

    var motorConfigs = new MotorOutputConfigs();
    motorConfigs.NeutralMode = NeutralModeValue.Brake;
    motorConfigs.Inverted = InvertedValue.Clockwise_Positive; // Change on testing
    motorTalonFXConfigurator.apply(motorConfigs);
  }

  public double getThroughPosition() {
    return throughBore.getAbsolutePosition().getValueAsDouble();
  }

  public double getMotorPosition() {
    return turretMotor.getPosition().getValueAsDouble();
  }

  //USE drive.getPose() to get
  public void moveToPos(double pos) {
    if (pos > 0.0 && pos <= Constants.Shooter.Turret.MAX_TURRET_ROTATION) { //Expand to 0.25 later
      applyMainConfigs();
      final MotionMagicVoltage mmRequest = new MotionMagicVoltage(pos);
      turretMotor.setControl(mmRequest.withPosition(pos));
    } else {
      double newPos = 0.0;
      if (pos - 0.23 > 0) {
        newPos = Constants.Shooter.Turret.MAX_TURRET_ROTATION;
      } else {
        newPos = 0.0;
      }
      final MotionMagicVoltage mmRequest = new MotionMagicVoltage(newPos);
      turretMotor.setControl(mmRequest.withPosition(newPos)); //Set to home to reset for a bit
    }
  }

  public void stopMotor() {
    turretMotor.setVoltage(0);
  }

  /**
   * Computes the virtual hub target that accounts for robot velocity so a ball
   * shot while moving still lands in the hub.
   *
   * <p>Algorithm (iterated twice for accuracy):
   * <ol>
   *   <li>Estimate flight time: {@code t = dist / ballSpeed}
   *   <li>Shift hub by robot velocity: {@code virtualTarget = hub - v * t}
   * </ol>
   *
   * @param robotPose current (offset-corrected) robot pose
   * @return field-relative Translation2d of the virtual target
   */
  private Translation2d getVirtualTarget(Pose2d robotPose, double vx, double vy) {
    boolean isBlue = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
        == DriverStation.Alliance.Blue;
    double hubX = isBlue ? BLUE_HUB_X : RED_HUB_X;
    double hubY = isBlue ? BLUE_HUB_Y : RED_HUB_Y;

    double vtX = hubX;
    double vtY = hubY;

    for (int iter = 0; iter < 2; iter++) {
      double dx = vtX - robotPose.getX();
      double dy = vtY - robotPose.getY();
      double dist = Math.hypot(dx, dy);
      if (dist < 0.01) break;

      // Convert interpolated flywheel RPS → ball surface speed (m/s)
      double flywheelRPS = Math.abs(Drive.flywheelSpeeds.get(dist));
      double ballSpeedMS = flywheelRPS * 2.0 * Math.PI * Constants.Shooter.Flywheel.FLYWHEEL_WHEEL_RADIUS_METERS;
      if (ballSpeedMS < 0.01) break;

      double t = dist / ballSpeedMS;
      vtX = hubX - vx * t;
      vtY = hubY - vy * t;
    }

    return new Translation2d(vtX, vtY);
  }

  // Parameterized to aim at any field-relative target point (virtual or real hub).
  private double getBlueHubRotate(Pose2d curPose, Translation2d target) {
    double xDIFF = target.getX() - curPose.getX();
    double yDIFF = target.getY() - curPose.getY();
    double radiansRotate = -Math.atan(yDIFF / xDIFF);

    double finalRadiansRotate = radiansRotate + curPose.getRotation().getRadians();
    double finalAngleRotate = Math.toDegrees(finalRadiansRotate);
    return (finalAngleRotate + 135);// * (0.46/0.5);
  }

  private double getRedHubRotate(Pose2d curPose, Translation2d target) {
    double xDIFF = target.getX() - curPose.getX();
    double yDIFF = target.getY() - curPose.getY();
    double radiansRotate = -Math.atan(yDIFF / xDIFF);

    double finalRadiansRotate = radiansRotate - normalizeRedRot(curPose.getRotation().getRadians());
    double finalAngleRotate = Math.toDegrees(finalRadiansRotate);

    return finalAngleRotate; //NEED TO TEST AND FIGURE OUT IF ITS +- 45!!!
  }

  private double normalizeRedRot(double input) {
    double output = -(input - Math.PI);
    if (output > Math.PI) {
        output -= 2 * Math.PI;
    }
    return output;
  }

  public double getTurretTurnPos() {
    Pose2d currentPose = drive.getPose(); //NEED TO ADD OFFSET FOR TURRET POSITION!!!
    currentPose = currentPose.plus(new Transform2d(
        new Translation2d(Constants.Shooter.Turret.OFFSET_POS_X, Constants.Shooter.Turret.OFFSET_POS_Y)
            .rotateBy(new Rotation2d(drive.getPose().getRotation().getRadians())),
        new Rotation2d(0)));

    // Use the cached virtual target (updated each periodic). Fall back to real hub if not yet set.
    Translation2d target = cachedVirtualTarget;
    if (target == null) {
      boolean isBlue = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
          == DriverStation.Alliance.Blue;
      target = new Translation2d(isBlue ? BLUE_HUB_X : RED_HUB_X, isBlue ? BLUE_HUB_Y : RED_HUB_Y);
    }

    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) return getRedHubRotate(currentPose, target);
    return getBlueHubRotate(currentPose, target);
  }

  /** Returns the distance from the turret origin to the cached virtual target (meters). */
  public double getVirtualTargetDistance() {
    return cachedVirtualTargetDistance;
  }

  /**
   * Returns the robot's field-relative velocity projected onto the turret→hub vector (m/s).
   * Positive = moving toward the hub. Used to correct flywheel speed.
   */
  public double getVelocityComponentTowardHub() {
    return cachedVelocityTowardHub;
  }

  /** Returns the last computed virtual target in field coordinates. */
  public Translation2d getLastVirtualTarget() {
    return cachedVirtualTarget;
  }

  @Override
  public void periodic() {
    //printRAWPositionForOffset();
    //System.out.println("GOAL: " + Constants.Shooter.Turret.TURRET_HOME_POS + "; END: " + getMotorPosition() + "; DIFF" + Math.abs(Constants.Shooter.Turret.TURRET_HOME_POS - getMotorPosition()));
    throughBorePosition.set(getThroughPosition());
    motorPosition.set(getMotorPosition());

    // --- Shoot-on-the-move: get field-relative velocity from Pigeon-backed drive ---
    ChassisSpeeds fieldSpeeds = drive.getFieldRelativeSpeeds();
    double vx = fieldSpeeds.vxMetersPerSecond;
    double vy = fieldSpeeds.vyMetersPerSecond;

    Pose2d currentPose = drive.getPose();

    // Compute offset-corrected turret origin pose (same as getTurretTurnPos)
    Pose2d turretPose = currentPose.plus(new Transform2d(
        new Translation2d(Constants.Shooter.Turret.OFFSET_POS_X, Constants.Shooter.Turret.OFFSET_POS_Y)
            .rotateBy(new Rotation2d(currentPose.getRotation().getRadians())),
        new Rotation2d(0)));

    // Cache virtual target and derived values for use by commands this cycle
    cachedVirtualTarget = getVirtualTarget(turretPose, vx, vy);
    double dx = cachedVirtualTarget.getX() - turretPose.getX();
    double dy = cachedVirtualTarget.getY() - turretPose.getY();
    cachedVirtualTargetDistance = Math.hypot(dx, dy);

    // Project robot velocity onto the turret→virtual-target unit vector
    if (cachedVirtualTargetDistance > 0.01) {
      cachedVelocityTowardHub = (vx * dx + vy * dy) / cachedVirtualTargetDistance;
    } else {
      cachedVelocityTowardHub = 0.0;
    }

    wallahi++;
    if (wallahi % 100 == 0) {
        getTurretTurnPos();
    } else if (wallahi > 2_000_000) wallahi = 0;

    //System.out.println("throughbore: " + throughBore.getAbsolutePosition().getValueAsDouble());
  }

  
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}