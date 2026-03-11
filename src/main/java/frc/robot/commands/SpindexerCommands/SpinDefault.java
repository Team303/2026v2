package frc.robot.commands.SpindexerCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Spindexer;

public class SpinDefault extends Command{
    
    Spindexer spindexer;

    public SpinDefault(Spindexer spindexer) {
        this.spindexer = spindexer;
        addRequirements(spindexer);
    }

   public void execute(){
        spindexer.spindexerMotor.set(0);
    }
}
