package frc.robot.commands.SpindexerCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Spindexer;

public class SpinForward extends Command{
    
    boolean forward;

    public SpinForward(boolean forward) {
        this.forward = forward;
        addRequirements(RobotContainer.spindexer);
    }

    public void initialize(){

    }

    public void execute(){
        if(forward){
            RobotContainer.spindexer.spindexerMotor.set(0.5);
            RobotContainer.spindexer.kickerMotor.set(0.5);
        }
        else{
            RobotContainer.spindexer.spindexerMotor.set(-0.5);
            RobotContainer.spindexer.kickerMotor.set(-0.5); 
        }
    }

    public void end(){
        RobotContainer.spindexer.spindexerMotor.set(0);
        RobotContainer.spindexer.kickerMotor.set(0);
    }
}
