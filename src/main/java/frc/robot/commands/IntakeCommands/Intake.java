package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeBelt;

public class Intake extends Command {
    
    boolean forward; 
    IntakeBelt intakeBelt;

    public Intake(IntakeBelt intakebelt, boolean forward) {
        addRequirements(intakebelt);
        this.intakeBelt = intakebelt;
        this.forward = forward;
    }
    public void initialize(){
        
    }
    public void execute(){
        if (forward){
            intakeBelt.beltMotor.set(0.5);
        }
        else {
            intakeBelt.beltMotor.set(-0.5);
        }
        
    }
    public void end(){
        intakeBelt.beltMotor.set(0);
    }
}


