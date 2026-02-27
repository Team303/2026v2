package frc.robot.commands.HoodCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Spindexer;

public class HoodDefault extends Command{
    
    Hood hood;

    public HoodDefault(Hood hood) {
        this.hood = hood;
        addRequirements(hood);
    }

   public void execute(){
        hood.moveToPos(0);
    }
}
