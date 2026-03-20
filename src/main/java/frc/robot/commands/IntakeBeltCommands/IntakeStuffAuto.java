package frc.robot.commands.IntakeBeltCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeBelt;

public class IntakeStuffAuto extends Command {
    
    IntakeBelt intakeBelt;

    public IntakeStuffAuto(IntakeBelt intakebelt) {
        addRequirements(intakebelt);
        this.intakeBelt = intakebelt;
        
    }
    public void initialize(){
        
    }
    public void execute(){
        intakeBelt.beltMotor.set(-1);
        intakeBelt.takeMotor.set(-0.75);
        
    }
    public void end(){
        intakeBelt.beltMotor.set(0);
    }
}


