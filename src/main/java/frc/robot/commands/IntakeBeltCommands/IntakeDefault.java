package frc.robot.commands.IntakeBeltCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeBelt;


public class IntakeDefault extends Command{
    
    IntakeBelt take;

    public IntakeDefault(IntakeBelt take) {
        this.take = take;
        addRequirements(take);
    }

   public void execute(){
        take.beltMotor.set(0);
        take.takeMotor.set(0);
    }
}
