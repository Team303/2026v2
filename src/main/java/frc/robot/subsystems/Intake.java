// package frc.robot.subsystems;

// import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

// import com.ctre.phoenix6.configs.CANcoderConfiguration;
// import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
// import com.ctre.phoenix6.configs.MotorOutputConfigs;
// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.controls.MotionMagicVoltage;
// import com.ctre.phoenix6.hardware.CANcoder;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.InvertedValue;
// import com.ctre.phoenix6.signals.NeutralModeValue;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;
// import frc.robot.util.LoggedTunableNumber;

// public class Intake extends SubsystemBase{
//     private final TalonFX leftIntakeMotor;
//     private final TalonFX rightIntakeMotor;
//     private final CANcoder intakeCaNcoder;

//     private final  LoggedNetworkNumber motorPosition;
//   private final LoggedNetworkNumber throughBorePosition;

//   // public static LoggedTunableNumber TESTING_kP =
//   //     new LoggedTunableNumber("intake TESTING_kP", Constants.Intake.INTAKE_kP);
//   // public static LoggedTunableNumber TESTING_kI =
//   //     new LoggedTunableNumber("intake TESTING_kI", Constants.Intake.INTAKE_kI);
//   // public static LoggedTunableNumber TESTING_kD =
//   //     new LoggedTunableNumber("intake TESTING_kD", Constants.Intake.INTAKE_kD);
//   // public static LoggedTunableNumber TESTING_kA =
//   //     new LoggedTunableNumber("intake TESTING_kA", Constants.Intake.INTAKE_kA);
//   // public static LoggedTunableNumber TESTING_mmV =
//   //     new LoggedTunableNumber("intake TESTING_mmV", Constants.Intake.INTAKE_maxV);
//   // public static LoggedTunableNumber TESTING_mmA =
//   //     new LoggedTunableNumber("intake TESTING_mmA", Constants.Intake.INTAKE_maxA);


//     public Intake() {
//       //  intakeCaNcoder = new CANcoder(Constants.Intake.INTAKE_THROUGHBORE_ID);

//         throughBorePosition = new LoggedNetworkNumber("Intake/ThroughBorePosition");
//         motorPosition = new LoggedNetworkNumber("intake/motorPosition");

//         leftIntakeMotor = null; //new TalonFX(Constants.Intake.LEFT_MOTOR_ID);
//         rightIntakeMotor = null;// new TalonFX(Constants.Intake.RIGHT_MOTOR_ID);

//         // leftElevatorMotor.setInverted(true);
//         // rightElevatorMotor.setInverted(false);
//         var TalonFXConfiguration = new TalonFXConfiguration();

//         var Slot0Configs = TalonFXConfiguration.Slot0;
//         var Slot1Configs = TalonFXConfiguration.Slot1;
//         Slot0Configs.kS = 0;
//         Slot1Configs.kS = Slot0Configs.kS;
//         Slot0Configs.kG = 0.40;
//         Slot1Configs.kG = Slot0Configs.kG;
//         Slot0Configs.kV = -0.3;
//         Slot1Configs.kV = -Slot0Configs.kV;
//         Slot0Configs.kA = -0.1;
//         Slot1Configs.kA = Slot0Configs.kA;
//         Slot0Configs.kP = 14 * 0.85;
//         Slot1Configs.kP = Slot0Configs.kP;
//         Slot0Configs.kI = 0;
//         Slot1Configs.kI = Slot0Configs.kI;
//         Slot0Configs.kD = 0;
//         Slot1Configs.kD = Slot0Configs.kD;

//         var motionMagicConfigs = TalonFXConfiguration.MotionMagic;
//         motionMagicConfigs.MotionMagicAcceleration = 35;
//         motionMagicConfigs.MotionMagicCruiseVelocity = 60;
//         leftIntakeMotor.getConfigurator().apply(TalonFXConfiguration);
//         rightIntakeMotor.getConfigurator().apply(TalonFXConfiguration);

//         leftIntakeMotor.setPosition(0);
//         rightIntakeMotor.setPosition(0);
//         leftIntakeMotor.getConfigurator().apply(TalonFXConfiguration);
//         rightIntakeMotor.getConfigurator().apply(TalonFXConfiguration);

//         var LeftElevatorMotorTalonFXConfigurator = leftIntakeMotor.getConfigurator();
//         var RightElevatorMotorTalonFXConfigurator = rightIntakeMotor.getConfigurator();

//         var limitConfigs = new CurrentLimitsConfigs();
//         var leftMotorConfigs = new MotorOutputConfigs();
//         var rightMotorConfigs = new MotorOutputConfigs();
//         leftMotorConfigs.Inverted = InvertedValue.Clockwise_Positive;
//         rightMotorConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
//         leftMotorConfigs.NeutralMode = NeutralModeValue.Brake;
//         rightMotorConfigs.NeutralMode = NeutralModeValue.Brake;

//         // enable stator current limit
//         limitConfigs.StatorCurrentLimit = 120;
//         limitConfigs.StatorCurrentLimitEnable = true;

//         limitConfigs.SupplyCurrentLimit = 120;
//         limitConfigs.SupplyCurrentLimitEnable = true;

//         LeftElevatorMotorTalonFXConfigurator.apply(leftMotorConfigs);
//         RightElevatorMotorTalonFXConfigurator.apply(rightMotorConfigs);
//         LeftElevatorMotorTalonFXConfigurator.apply(limitConfigs);
//         RightElevatorMotorTalonFXConfigurator.apply(limitConfigs);

     
        
//     }  
    
//     public void createNewConfig() {
//     var hoodMotorConfig = new TalonFXConfiguration();

//     //hoodMotorConfig.Feedback.FeedbackRemoteSensorID = throughBore.getDeviceID();
//     //hoodMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
//     //hoodMotorConfig.Feedback.SensorToMechanismRatio = 1.0;
//     //hoodMotorConfig.Feedback.RotorToSensorRatio = 1.0; //NEED TO CHANGE

//     var Slot0Configs = hoodMotorConfig.Slot0;
//     Slot0Configs.kS = Constants.Shooter.Hood.HOOD_kS;
//     Slot0Configs.kP = Constants.Shooter.Hood.HOOD_kP;
//     Slot0Configs.kI = Constants.Shooter.Hood.HOOD_kI;
//     Slot0Configs.kD = Constants.Shooter.Hood.HOOD_kD;

//     var motionMagicConfigs = hoodMotorConfig.MotionMagic;
//     motionMagicConfigs.MotionMagicCruiseVelocity = Constants.Shooter.Hood.HOOD_maxV;
//     motionMagicConfigs.MotionMagicAcceleration = Constants.Shooter.Hood.HOOD_maxA;
//     leftIntakeMotor.getConfigurator().apply(hoodMotorConfig);
//     rightIntakeMotor.getConfigurator().apply(hoodMotorConfig);
//   }

// public double getThroughPosition() {
//     return intakeCaNcoder.getAbsolutePosition().getValueAsDouble();
//   }

//   public double getLeftMotorPosition() {
//     return leftIntakeMotor.getPosition().getValueAsDouble();
//   }

//    public double getRightMotorPosition() {
//     return rightIntakeMotor.getPosition().getValueAsDouble();
//   }

//     public void moveToPos(double pos) {
//          final MotionMagicVoltage m_request = new MotionMagicVoltage(pos);
//         // System.out.println(targetPos);
//         leftIntakeMotor.setControl(m_request.withPosition((pos)).withSlot(1));
//         rightIntakeMotor.setControl(m_request.withPosition((pos)).withSlot(1));
//     }

//     public void stopMotors() {
//         leftIntakeMotor.set(0);
//         rightIntakeMotor.set(0);
//     }

//     @Override
//   public void periodic() {
//     // throughBorePosition.set(getThroughPosition());
//     // motorPosition.set(getMotorPosition());
//   }

// }
