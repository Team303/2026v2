package frc.robot.commands.HoodCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hood;
import static frc.robot.RobotContainer.turret;

public class RotateToPosition extends Command {
  //private final double GOAL_THRESHOLD = 0 / 360.0;
  private double goal;
  private Hood hood;
 // private Drive drive;

  public RotateToPosition(Hood hood) {
    //addRequirements(drive, hood);
    this.goal = 0;
  //  this.drive = drive;
    this.hood = hood;
  }

  @Override
  public void initialize() {
    goal = hood.getShootOnMoveAngle(turret.getVirtualTargetDistance());
  }

  @Override
  public void execute() {
    goal = hood.getShootOnMoveAngle(turret.getVirtualTargetDistance());
    hood.moveToPos(goal);
    System.out.println("hood goal: " + goal);


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