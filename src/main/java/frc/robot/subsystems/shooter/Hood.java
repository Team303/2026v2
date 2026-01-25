package frc.robot.subsystems.shooter;

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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Hood extends SubsystemBase {

  private final TalonFX hoodMotor;
  //private final CANcoder throughBore; // Powered by CANCoder

  private final LoggedNetworkNumber motorPosition;
  private final LoggedNetworkNumber throughBorePosition;

    public static LoggedTunableNumber TESTING_kP =
        new LoggedTunableNumber("HOOD TESTING_kP", Constants.Shooter.Hood.HOOD_kP);
    public static LoggedTunableNumber TESTING_kI =
        new LoggedTunableNumber("HOOD TESTING_kI", Constants.Shooter.Hood.HOOD_kI);
    public static LoggedTunableNumber TESTING_kD =
        new LoggedTunableNumber("HOOD TESTING_kD", Constants.Shooter.Hood.HOOD_kD);
    public static LoggedTunableNumber TESTING_mmV =
        new LoggedTunableNumber("HOOD TESTING_mmV", Constants.Shooter.Hood.HOOD_maxV);
    public static LoggedTunableNumber TESTING_mmA =
        new LoggedTunableNumber("HOOD TESTING_mmA", Constants.Shooter.Hood.HOOD_maxA);

    public static LoggedTunableNumber GOAL_POS = new LoggedTunableNumber("HOOD GOAL_POS", Constants.Shooter.Hood.HOOD_HOME_POS);


  public Hood() {
    //throughBore = new CANcoder(Constants.Shooter.Hood.HOOD_THROUGHBORE_ID);
    CANcoderConfiguration cc_cfg = new CANcoderConfiguration();
    cc_cfg.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
    //throughBore.getConfigurator().apply(cc_cfg);

    hoodMotor = new TalonFX(Constants.Shooter.Hood.HOOD_MOTOR_ID);

    var hoodMotorConfig = new TalonFXConfiguration();

    //hoodMotorConfig.Feedback.FeedbackRemoteSensorID = throughBore.getDeviceID();
    //hoodMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    //hoodMotorConfig.Feedback.SensorToMechanismRatio = 1.0;
    //hoodMotorConfig.Feedback.RotorToSensorRatio = 1.0; //NEED TO CHANGE

    var Slot0Configs = hoodMotorConfig.Slot0;
    Slot0Configs.kS = Constants.Shooter.Hood.HOOD_kS;
    Slot0Configs.kP = Constants.Shooter.Hood.HOOD_kP;
    Slot0Configs.kI = Constants.Shooter.Hood.HOOD_kI;
    Slot0Configs.kD = Constants.Shooter.Hood.HOOD_kD;

    var motionMagicConfigs = hoodMotorConfig.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = Constants.Shooter.Hood.HOOD_maxV;
    motionMagicConfigs.MotionMagicAcceleration = Constants.Shooter.Hood.HOOD_maxA;
    hoodMotor.getConfigurator().apply(hoodMotorConfig);

    var motorTalonFXConfigurator = hoodMotor.getConfigurator();

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

    throughBorePosition = new LoggedNetworkNumber("Hood Absolute Position", 0.0);
    motorPosition = new LoggedNetworkNumber("Hood Motor Position", 0.0);
  }

  public void createNewConfig() {
    var hoodMotorConfig = new TalonFXConfiguration();

    //hoodMotorConfig.Feedback.FeedbackRemoteSensorID = throughBore.getDeviceID();
    //hoodMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    //hoodMotorConfig.Feedback.SensorToMechanismRatio = 1.0;
    //hoodMotorConfig.Feedback.RotorToSensorRatio = 1.0; //NEED TO CHANGE

    var Slot0Configs = hoodMotorConfig.Slot0;
    Slot0Configs.kS = Constants.Shooter.Hood.HOOD_kS;
    Slot0Configs.kP = Constants.Shooter.Hood.HOOD_kP;
    Slot0Configs.kI = Constants.Shooter.Hood.HOOD_kI;
    Slot0Configs.kD = Constants.Shooter.Hood.HOOD_kD;

    var motionMagicConfigs = hoodMotorConfig.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = Constants.Shooter.Hood.HOOD_maxV;
    motionMagicConfigs.MotionMagicAcceleration = Constants.Shooter.Hood.HOOD_maxA;
    hoodMotor.getConfigurator().apply(hoodMotorConfig);
  }

  public double getThroughPosition() {
    return 0.0;//throughBore.getAbsolutePosition().getValueAsDouble();
  }

  public double getMotorPosition() {
    return hoodMotor.getPosition().getValueAsDouble();
  }

  public void moveToPos(double pos) {
    final MotionMagicVoltage mmRequest = new MotionMagicVoltage(pos);
    hoodMotor.setControl(mmRequest.withPosition(pos));
  }

  public void stopMotor() {
    hoodMotor.setVoltage(0);
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