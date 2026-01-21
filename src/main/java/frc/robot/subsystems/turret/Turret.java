package frc.robot.subsystems.turret;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Turret extends SubsystemBase {

  private final TalonFX turretMotor;
  private final CANcoder throughBore; // Powered by CANCoder

  private final LoggedNetworkNumber relativePosition;
  private final LoggedNetworkNumber absolutePosition;
  private final LoggedNetworkNumber motorPosition;

  public Turret() {
    throughBore = new CANcoder(Constants.Shooter.TURRET_THROUGHBORE_ID);
    turretMotor = new TalonFX(Constants.Shooter.TURRET_MOTOR_ID);

    var turretMotorConfig = new TalonFXConfiguration();

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

    relativePosition = new LoggedNetworkNumber("Relative Position", 0.0);
    absolutePosition = new LoggedNetworkNumber("Relative Positin", 0.0);
    motorPosition = new LoggedNetworkNumber("Motor Position", 0.0);
  }

  public double getAbsolutePosition() {
    return throughBore.getAbsolutePosition().getValueAsDouble();
  }

  public double getRelativePosition() {
    return throughBore.getPosition().getValueAsDouble();
  }

  public double getMotorPosition() {
    return turretMotor.getPosition().getValueAsDouble();
  }

  public void setPosition(double position) {
    throughBore.setPosition(position);
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
    // This method will be called once per scheduler run
    relativePosition.set(getRelativePosition());
    absolutePosition.set(getAbsolutePosition());
    motorPosition.set(getMotorPosition());
    System.out.printf("Rel - %f", getRelativePosition());
    System.out.printf("Abs - %f", getAbsolutePosition());
    System.out.printf("Mot - %f", getMotorPosition());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
