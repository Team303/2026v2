package frc.robot.subsystems.turret;

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

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Turret extends SubsystemBase {

  public final TalonFX turretMotor;
  public final CANcoder throughBore; // Powered by CANCoder

  private final LoggedNetworkNumber motorPosition;
  private final LoggedNetworkNumber throughBorePosition;

    public static LoggedTunableNumber TESTING_kP =
        new LoggedTunableNumber("TESTING_kP", Constants.Shooter.TURRET_kP);
    public static LoggedTunableNumber TESTING_kI =
        new LoggedTunableNumber("TESTING_kI", Constants.Shooter.TURRET_kI);
    public static LoggedTunableNumber TESTING_kD =
        new LoggedTunableNumber("TESTING_kD", Constants.Shooter.TURRET_kD);
    public static LoggedTunableNumber TESTING_kA =
        new LoggedTunableNumber("TESTING_kA", Constants.Shooter.TURRET_kA);
    public static LoggedTunableNumber TESTING_mmV =
        new LoggedTunableNumber("TESTING_mmV", Constants.Shooter.TURRET_mmV);
    public static LoggedTunableNumber TESTING_mmA =
        new LoggedTunableNumber("TESTING_mmA", Constants.Shooter.TURRET_mmA);

    public static LoggedTunableNumber GOAL_POS = new LoggedTunableNumber("GOAL_POS", Constants.Shooter.TEST_HUB_POS);


  public Turret() {
    throughBore = new CANcoder(Constants.Shooter.TURRET_THROUGHBORE_ID);
    CANcoderConfiguration cc_cfg = new CANcoderConfiguration();
    cc_cfg.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
    throughBore.getConfigurator().apply(cc_cfg);

    turretMotor = new TalonFX(Constants.Shooter.TURRET_MOTOR_ID);

    var turretMotorConfig = new TalonFXConfiguration();

    turretMotorConfig.Feedback.FeedbackRemoteSensorID = throughBore.getDeviceID();
    turretMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    turretMotorConfig.Feedback.SensorToMechanismRatio = 1.0;
    turretMotorConfig.Feedback.RotorToSensorRatio = 2.8;

    var Slot0Configs = turretMotorConfig.Slot0;
    Slot0Configs.kP = Constants.Shooter.TURRET_kP;
    Slot0Configs.kI = Constants.Shooter.TURRET_kI;
    Slot0Configs.kD = Constants.Shooter.TURRET_kD;
    Slot0Configs.kA = Constants.Shooter.TURRET_kA;

    var motionMagicConfigs = turretMotorConfig.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = Constants.Shooter.TURRET_mmV;
    motionMagicConfigs.MotionMagicAcceleration = Constants.Shooter.TURRET_mmA;
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

    throughBorePosition = new LoggedNetworkNumber("Absolute Position", 0.0);
    motorPosition = new LoggedNetworkNumber("Motor Position", 0.0);
  }

  public void createNewConfig() {
    var turretMotorConfig = new TalonFXConfiguration();

    turretMotorConfig.Feedback.FeedbackRemoteSensorID = throughBore.getDeviceID();
    turretMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    turretMotorConfig.Feedback.SensorToMechanismRatio = 1.0;
    turretMotorConfig.Feedback.RotorToSensorRatio = 2.8;

    var Slot0Configs = turretMotorConfig.Slot0;
    Slot0Configs.kS = 0.4;
    Slot0Configs.kP = TESTING_kP.getAsDouble();
    Slot0Configs.kI = TESTING_kI.getAsDouble();
    Slot0Configs.kD = TESTING_kD.getAsDouble();
    Slot0Configs.kA = TESTING_kA.getAsDouble();

    var motionMagicConfigs = turretMotorConfig.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = TESTING_mmV.getAsDouble();
    motionMagicConfigs.MotionMagicAcceleration = TESTING_mmA.getAsDouble();
    turretMotor.getConfigurator().apply(turretMotorConfig);
  }

  public double getThroughPosition() {
    return throughBore.getAbsolutePosition().getValueAsDouble();
  }

  public double getMotorPosition() {
    return turretMotor.getPosition().getValueAsDouble();
  }

  public void moveToPos(double pos) {
    final MotionMagicVoltage mmRequest = new MotionMagicVoltage(pos);
    turretMotor.setControl(mmRequest.withPosition(pos));
  }

  public void stopMotor() {
    turretMotor.setVoltage(0);
  }

  @Override
  public void periodic() {
    throughBorePosition.set(getThroughPosition());
    motorPosition.set(getMotorPosition());

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}