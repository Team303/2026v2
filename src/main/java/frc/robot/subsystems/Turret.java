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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;

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

  private static final double BLUE_HUB_X = 4.6269;
  private static final double BLUE_HUB_Y = 4.03;
  private static final double RED_HUB_X = 11.91358;
  private static final double RED_HUB_Y = 4.03;

  public Turret() {
    throughBore = new CANcoder(Constants.Shooter.Turret.TURRET_THROUGHBORE_ID, "topside");
    CANcoderConfiguration cc_cfg = new CANcoderConfiguration();
    cc_cfg.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
    cc_cfg.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    cc_cfg.MagnetSensor.MagnetOffset = -0.491455078125;
    throughBore.getConfigurator().apply(cc_cfg);

    turretMotor = new TalonFX(Constants.Shooter.Turret.TURRET_MOTOR_ID, "topside");

    applyMainConfigs();

    throughBorePosition = new LoggedNetworkNumber("Turret Absolute Position", 0.0);
    motorPosition = new LoggedNetworkNumber("Turret Motor Position", 0.0);
    
    wallahi = 0;
  }

  private void applyMainConfigs() {
    var turretMotorConfig = new TalonFXConfiguration();

    turretMotorConfig.Feedback.FeedbackRemoteSensorID = throughBore.getDeviceID();
    turretMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    turretMotorConfig.Feedback.SensorToMechanismRatio = 85.0 / 10.0;
    turretMotorConfig.Feedback.FeedbackRotorOffset = 0.056884765625;
    //turretMotorConfig.Feedback.RotorToSensorRatio = Constants.Shooter.Turret.TURRET_MOTOR_THROUGHBORE_RATIO;

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
    limitConfigs.StatorCurrentLimit = 120;
    limitConfigs.SupplyCurrentLimit = 120;
    limitConfigs.StatorCurrentLimitEnable = true;
    limitConfigs.SupplyCurrentLimitEnable = true;
    motorTalonFXConfigurator.apply(limitConfigs);

    var motorConfigs = new MotorOutputConfigs();
    motorConfigs.NeutralMode = NeutralModeValue.Brake;
    motorConfigs.Inverted = InvertedValue.Clockwise_Positive; // Change on testing
    motorTalonFXConfigurator.apply(motorConfigs);
  }

  private void applySafeConfigs() {
    var turretMotorConfig = new TalonFXConfiguration();

    turretMotorConfig.Feedback.FeedbackRemoteSensorID = throughBore.getDeviceID();
    turretMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    turretMotorConfig.Feedback.SensorToMechanismRatio = 85.0 / 10.0;
    turretMotorConfig.Feedback.FeedbackRotorOffset = 0.056884765625;
    //turretMotorConfig.Feedback.RotorToSensorRatio = Constants.Shooter.Turret.TURRET_MOTOR_THROUGHBORE_RATIO;

    var Slot0Configs = turretMotorConfig.Slot0;
    Slot0Configs.kS = Constants.Shooter.Turret.TURRET_kS;
    Slot0Configs.kP = Constants.Shooter.Turret.TURRET_kP;
    Slot0Configs.kI = Constants.Shooter.Turret.TURRET_kI;
    Slot0Configs.kD = Constants.Shooter.Turret.TURRET_kD;
    Slot0Configs.kA = Constants.Shooter.Turret.TURRET_kA;

    var motionMagicConfigs = turretMotorConfig.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = Constants.Shooter.Turret.TURRET_maxV / 4;
    motionMagicConfigs.MotionMagicAcceleration = Constants.Shooter.Turret.TURRET_maxA / 8;
    turretMotor.getConfigurator().apply(turretMotorConfig);

    var motorTalonFXConfigurator = turretMotor.getConfigurator();

    var limitConfigs = new CurrentLimitsConfigs();
    limitConfigs.StatorCurrentLimit = 40;
    limitConfigs.SupplyCurrentLimit = 40;
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
    if (Math.abs(pos) < Constants.Shooter.Turret.MAX_TURRET_ROTATION) { //Expand to 0.25 later
      applyMainConfigs();
      final MotionMagicVoltage mmRequest = new MotionMagicVoltage(pos);
      turretMotor.setControl(mmRequest.withPosition(pos));
    } else {
      applySafeConfigs();
      final MotionMagicVoltage mmRequest = new MotionMagicVoltage(pos);
      turretMotor.setControl(mmRequest.withPosition(0)); //Set to home to reset for a bit
    }
  }

  public void stopMotor() {
    turretMotor.setVoltage(0);
  }

  private double getBlueHubRotate(Pose2d curPose) {
    double xDIFF = BLUE_HUB_X - curPose.getX();
    double yDIFF = BLUE_HUB_Y - curPose.getY();
    double radiansRotate = -Math.atan(yDIFF / xDIFF);

    double finalRadiansRotate = radiansRotate + curPose.getRotation().getRadians();
    double finalAngleRotate = Math.toDegrees(finalRadiansRotate);
    return finalAngleRotate;
  }

  private double getRedHubRotate(Pose2d curPose) {
    double xDIFF = RED_HUB_X - curPose.getX();
    double yDIFF = RED_HUB_Y - curPose.getY(); 
    double radiansRotate = -Math.atan(yDIFF / xDIFF);
    
    double finalRadiansRotate = radiansRotate - normalizeRedRot(curPose.getRotation().getRadians());
    double finalAngleRotate = Math.toDegrees(finalRadiansRotate);

    return finalAngleRotate;
  }

  private double normalizeRedRot(double input) {
    double output = -(input - Math.PI);
    if (output > Math.PI) {
        output -= 2 * Math.PI;
    }
    return output;
  }

  public double getTurretTurnPos() {
    Pose2d currentPose = drive.getPose();
    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) return getRedHubRotate(currentPose);
    return getBlueHubRotate(currentPose);
  }

  @Override
  public void periodic() {
    System.out.println("GOAL: " + Constants.Shooter.Turret.TURRET_HOME_POS + "; END: " + getMotorPosition() + "; DIFF" + Math.abs(Constants.Shooter.Turret.TURRET_HOME_POS - getMotorPosition()));
    throughBorePosition.set(getThroughPosition());
    motorPosition.set(getMotorPosition());
    wallahi++;
    if (wallahi % 100 == 0) {
        getTurretTurnPos();  
    } else if (wallahi > 2_000_000) wallahi = 0;
  }

  
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}