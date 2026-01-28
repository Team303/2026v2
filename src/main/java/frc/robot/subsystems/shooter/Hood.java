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
  private final CANcoder throughBore; // Powered by CANCoder

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


  public Hood() { //Deadass the exact same thing as the Turret (changes are in the parenthesis) - No Copy and Paste
    //1. Initalize CANCoder & Create CANcoderConfiguration (Discountinuty Point is 1)


    //2. Initalize Hood Motor with TalonFX ID

    //3. Create TalonFXConfiguration

    //4. Link the Feedback for the Hood Motor Config to the throughbore (Set SensorToMechanismRatio & RotorToSensorRatio to 1.0)


    //5. Create PID configs for Slot0 (kP, kI, kD, kS)
    

    //6. Create Motion Magic Configs inside the Hood Motor set it equal to the values in the Constants file under the Shooter class


    //7. Apply the config to the hoodMotor


    //8. Get the hoodMotor configurator


    //9. Create Current Limit Configs (Supply & Stator)
    

    //10. Apply Current Limit Configs


    //11. Create MotorOutputConfigs and apply Brake Mode for Neutral Mode and Clockwise_positive for Inverted
    

    //12. Apply the motorOutputConfigs to the configurator from step 8
    

    //13. Create the LoggedNetworkNumers for the throughBorePosition and motorPosition variables (make sure to set the key and default value as 0.0)
    
  }

  public void createNewConfig() {
  }

  public double getThroughPosition() {
    return 0.0; //0.0 temporarily
  }

  public double getMotorPosition() {
    return 0.0; //0.0 temporarily
  }

  public void moveToPos(double pos) {
  }

  public void stopMotor() {
  }

  @Override
  public void periodic() {
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}