package frc.robot.commands.HoodCommands;

import static frc.robot.Constants.Shooter.Hood.GOAL_POS;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hood;

public class RotateToPosition extends Command {
  private final double GOAL_THRESHOLD = 1 / 360.0;
  private double goal; //Rotations
  private Hood hood;

  public RotateToPosition(Hood hood, double goal) {
    addRequirements(hood);
    this.goal = goal;
    this.hood = hood;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    System.out.println("moving!");
    hood.moveToPos(goal);

    //hood.hoodMotor.setVoltage(-0.9);
  }

  // @Override
  // public boolean isFinished() {
  //   //System.out.println("isFinished Diff: " + Math.abs(goal - turret.throughBore.getPosition().getValueAsDouble()) + "; Thres: " + GOAL_THRESHOLD);
  //  return Math.abs(goal - hood.getMotorPosition()) < GOAL_THRESHOLD;
  // }

  // @Override
  // public void end(boolean interrupted) {
  //   System.out.println("GOAL: " + goal + "; END: " + hood.getMotorPosition() + "; DIFF" + Math.abs(goal - hood.getMotorPosition()));
  //   hood.stopMotors();
  // }
}