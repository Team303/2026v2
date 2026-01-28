package frc.robot.subsystems;


import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Spindexer extends SubsystemBase{
    public TalonFX spindexerMotor;
    public TalonFX kickerMotor;

    public Spindexer(){
        spindexerMotor = new TalonFX(Constants.Spindexer.SPINDEXER_MOTOR_ID);
        kickerMotor = new TalonFX(Constants.Spindexer.KICKER_MOTOR_ID);
    }
}
