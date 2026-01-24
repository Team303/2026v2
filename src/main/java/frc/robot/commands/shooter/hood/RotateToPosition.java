package frc.robot.commands.shooter.hood;

import static frc.robot.RobotContainer.hood;
import static frc.robot.subsystems.shooter.Hood.GOAL_POS;

import edu.wpi.first.wpilibj2.command.Command;

public class RotateToPosition extends Command {
  private final double GOAL_THRESHOLD = 1 / 360.0;
  private double goal; //Rotations

  public RotateToPosition(double goal) {
    addRequirements(hood);
    this.goal = GOAL_POS.getAsDouble();
  }

  @Override
  public void initialize() {
    hood.createNewConfig();
    this.goal = GOAL_POS.getAsDouble();
    System.out.println("THRES: " + GOAL_THRESHOLD);
  }

  @Override
  public void execute() {
    hood.moveToPos(goal);
  }

  @Override
  public boolean isFinished() {
    //System.out.println("isFinished Diff: " + Math.abs(goal - turret.throughBore.getPosition().getValueAsDouble()) + "; Thres: " + GOAL_THRESHOLD);
    return Math.abs(goal - hood.getMotorPosition()) < GOAL_THRESHOLD;
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("GOAL: " + goal + "; END: " + hood.getMotorPosition() + "; DIFF" + Math.abs(goal - hood.getMotorPosition()));
    hood.stopMotor();
  }
}