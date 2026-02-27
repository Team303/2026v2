package frc.robot.commands.SpindexerCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Spindexer;

public class spin2 extends Command{
    
    boolean forward;
    Spindexer spindexer;

    public spin2(Spindexer spindexer, boolean forward) {
        this.forward = forward;
        this.spindexer = spindexer;
        addRequirements(spindexer);
    }

    public void initialize(){

    }

    public void execute(){
        System.out.println("spinning");
        if(forward){
            spindexer.spindexerMotor.set(0.6);
        }
        else{
            spindexer.spindexerMotor.set(-0.6);
        }
    }

    public void end(){
     //   spindexer.spindexerMotor.set(0);
    }
}
