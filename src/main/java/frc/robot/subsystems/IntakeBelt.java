package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class IntakeBelt extends SubsystemBase{
    public TalonFX beltMotor;
    public IntakeBelt () {
        beltMotor = new TalonFX(Constants.IntakeBelt.INTAKEBELT_MOTOR_ID);
    }
}

