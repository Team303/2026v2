package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.Follower;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;
import frc.robot.util.LoggedTunableNumber;

public class Flywheel extends SubsystemBase {
    private final TalonFX leftFlywheelMotor;
    private final TalonFX rightFlywheelMotor;

    private final LoggedNetworkNumber leftMotorSpeed;
    private final LoggedNetworkNumber rightMotorSpeed;

    public static LoggedTunableNumber FLY_TESTING_kP =
        new LoggedTunableNumber("FLYWHEEL TESTING_kP", Constants.Shooter.Flywheel.FLYWHEEL_kP);
    public static LoggedTunableNumber FLY_TESTING_kI =
        new LoggedTunableNumber("FLYWHEEL TESTING_kI", Constants.Shooter.Flywheel.FLYWHEEL_kI);
    public static LoggedTunableNumber FLY_TESTING_kD =
        new LoggedTunableNumber("FLYWHEEL TESTING_kD", Constants.Shooter.Flywheel.FLYWHEEL_kD);
    public static LoggedTunableNumber FLY_TESTING_kV =
        new LoggedTunableNumber("FLYWHEEL TESTING_kV", Constants.Shooter.Flywheel.FLYWHEEL_kV);
    public static LoggedTunableNumber FLY_TESTING_mmV =
        new LoggedTunableNumber("FLYWHEEL TESTING_mmV", Constants.Shooter.Flywheel.FLYWHEEL_maxV);
    public static LoggedTunableNumber FLY_TESTING_mmA =
        new LoggedTunableNumber("FLYWHEEL TESTING_mmA", Constants.Shooter.Flywheel.FLYWHEEL_maxA);

    public static LoggedTunableNumber GOAL_SPEED = new LoggedTunableNumber("FLYWHEEL GOAL_SPEED", 0);

    public Flywheel() {
        leftFlywheelMotor = new TalonFX(Constants.Shooter.Flywheel.FLYWHEEL_LEFT_MOTOR_ID);
        rightFlywheelMotor = new TalonFX(Constants.Shooter.Flywheel.FLYWHEEL_RIGHT_MOTOR_ID);
        var flywheelMotorsConfig = new TalonFXConfiguration();

        var Slot0Configs = flywheelMotorsConfig.Slot0;
        Slot0Configs.kS = Constants.Shooter.Flywheel.FLYWHEEL_kS;
        Slot0Configs.kP = Constants.Shooter.Flywheel.FLYWHEEL_kP;
        Slot0Configs.kI = Constants.Shooter.Flywheel.FLYWHEEL_kI;
        Slot0Configs.kD = Constants.Shooter.Flywheel.FLYWHEEL_kD;
        Slot0Configs.kV = Constants.Shooter.Flywheel.FLYWHEEL_kV;

        var motionMagicConfigs = flywheelMotorsConfig.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = Constants.Shooter.Flywheel.FLYWHEEL_maxV;
        motionMagicConfigs.MotionMagicAcceleration = Constants.Shooter.Flywheel.FLYWHEEL_maxA;

        leftFlywheelMotor.getConfigurator().apply(flywheelMotorsConfig); //MASTER MOTOR

        var limitConfigs = new CurrentLimitsConfigs();
        limitConfigs.StatorCurrentLimit = 120;
        limitConfigs.SupplyCurrentLimit = 120;
        limitConfigs.StatorCurrentLimitEnable = true;
        limitConfigs.SupplyCurrentLimitEnable = true;
        leftFlywheelMotor.getConfigurator().apply(limitConfigs);
        rightFlywheelMotor.getConfigurator().apply(limitConfigs);

        var leftMotorConfigs = new MotorOutputConfigs();
        leftMotorConfigs.NeutralMode = NeutralModeValue.Brake;
        leftMotorConfigs.Inverted = InvertedValue.Clockwise_Positive; //NEED TO CHANGE WHEN SETUP
        leftFlywheelMotor.getConfigurator().apply(leftMotorConfigs);

        var rightMotorConfigs = new MotorOutputConfigs();
        rightMotorConfigs.NeutralMode = NeutralModeValue.Brake;
        rightMotorConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
        rightFlywheelMotor.getConfigurator().apply(rightMotorConfigs);

        leftMotorSpeed = new LoggedNetworkNumber("LEFT FLYWHEEL SPEED", 0.0);
        rightMotorSpeed = new LoggedNetworkNumber("RIGHT FLYWHEEL SPEED", 0.0);

        FLY_TESTING_kP = new LoggedTunableNumber("FLYWHEEL TESTING_kP", Constants.Shooter.Flywheel.FLYWHEEL_kP);
    }   


    public void createNewConfig() {
        var flywheelMotorsConfig = new TalonFXConfiguration();

        var Slot0Configs = flywheelMotorsConfig.Slot0;
        Slot0Configs.kS = Constants.Shooter.Flywheel.FLYWHEEL_kS;
        Slot0Configs.kP = FLY_TESTING_kP.getAsDouble();
        Slot0Configs.kI = FLY_TESTING_kI.getAsDouble();
        Slot0Configs.kD = FLY_TESTING_kD.getAsDouble();
        Slot0Configs.kV = FLY_TESTING_kV.getAsDouble();

        var motionMagicConfigs = flywheelMotorsConfig.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = FLY_TESTING_mmV.getAsDouble();;
        motionMagicConfigs.MotionMagicAcceleration = FLY_TESTING_mmA.getAsDouble();;

        leftFlywheelMotor.getConfigurator().apply(flywheelMotorsConfig); //MASTER MOTOR
        rightFlywheelMotor.getConfigurator().apply(flywheelMotorsConfig);
    }

    public double getLeftMotorSpeed() {
        return leftFlywheelMotor.getVelocity().getValueAsDouble();
    }

    public double getRightMotorSpeed() {
        return rightFlywheelMotor.getVelocity().getValueAsDouble();
    }

    public void getToSpeed(double speed) {
        final MotionMagicVelocityVoltage mmRequest = new MotionMagicVelocityVoltage(speed);
        //leftFlywheelMotor.setControl(mmRequest);
        rightFlywheelMotor.setControl(mmRequest);//new Follower(Constants.Shooter.Flywheel.FLYWHEEL_LEFT_MOTOR_ID, 
                                                 // MotorAlignmentValue.Opposed));
        leftFlywheelMotor.setControl(new Follower(Constants.Shooter.Flywheel.FLYWHEEL_RIGHT_MOTOR_ID,
                                        MotorAlignmentValue.Opposed));
    }

    public void stopMotors() {
        leftFlywheelMotor.setVoltage(0);
        rightFlywheelMotor.setVoltage(0);
    }

    @Override
    public void periodic() {
        leftMotorSpeed.set(getLeftMotorSpeed());
        rightMotorSpeed.set(getRightMotorSpeed());
        //Logger.recordOutput("Flywheel/LeftMotorSpeed", getLeftMotorSpeed());
        //Logger.recordOutput("Flywheel/RightMotorSpeed", getRightMotorSpeed());
        //Logger.recordOutput("Flywheel/LeftMotorVoltage", leftFlywheelMotor.getMotorVoltage().getValueAsDouble());
        //Logger.recordOutput("Flywheel/RightMotorVoltage", rightFlywheelMotor.getMotorVoltage().getValueAsDouble());
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
