package frc.robot.commands.SpindexerCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Spindexer;

public class SpinForward extends Command{
    
    boolean forward;
    Spindexer spindexer;

    public SpinForward(Spindexer spindexer, boolean forward) {
        this.forward = forward;
        this.spindexer = spindexer;
        addRequirements(spindexer);
    }

    public void initialize(){

    }

    public void execute(){
        System.out.println("spinning");
        if(forward){
            spindexer.kickerMotor.set(-1);
        }
        else{
            spindexer.kickerMotor.set(1); 
        }
    }

    public void end(){
        // spindexer.kickerMotor.set(0);
    }
}
