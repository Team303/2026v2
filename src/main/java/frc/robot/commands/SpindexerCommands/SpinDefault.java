package frc.robot.commands.SpindexerCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Spindexer;

public class SpinDefault extends Command{
    
    Spindexer spindexer;
    Hood hood;

    public SpinDefault(Spindexer spindexer) {
        this.spindexer = spindexer;
        addRequirements(spindexer);
    }

   public void execute(){
        spindexer.spindexerMotor.set(0);
        spindexer.kickerMotor.set(0);
    }
}
