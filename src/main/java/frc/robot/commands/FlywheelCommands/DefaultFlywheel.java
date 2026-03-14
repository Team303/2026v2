package frc.robot.commands.FlywheelCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Spindexer;

public class DefaultFlywheel extends Command{
    
    Flywheel hood;

    public 
    DefaultFlywheel(Flywheel hood) {
        this.hood = hood;
        addRequirements(hood);
    }

   public void execute(){
        hood.getToSpeed(0);
    }
}
