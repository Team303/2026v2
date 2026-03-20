package frc.robot.commands.SpindexerCommands;

import java.util.logging.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Spindexer;
import static frc.robot.subsystems.drive.Drive.dist;

public class spin2 extends Command{
    
    boolean forward;
    Spindexer spindexer;
    boolean fast;

    public spin2(Spindexer spindexer, boolean forward, boolean fast) {
        this.forward = forward;
        this.fast = fast;
        this.spindexer = spindexer;
        addRequirements(spindexer);
    }

    public void initialize(){

    }

    public void execute(){
        //System.out.println("spinning");
        if(forward){
            spindexer.spindexerMotor.set(fast ? 0.65 : 0.6);
        }
        else{
            spindexer.spindexerMotor.set(fast ? -0.65 : -0.6);
        }
    }

    public void end(){
     //   spindexer.spindexerMotor.set(0);
    }
}
