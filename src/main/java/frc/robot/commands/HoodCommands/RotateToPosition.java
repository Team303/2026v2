package frc.robot.commands.HoodCommands;

import static frc.robot.subsystems.Hood.HOOD_GOAL_POS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.drive.Drive;
import static frc.robot.RobotContainer.drive;

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
    goal = drive.calculateHoodAngle();
  }

  @Override
  public void execute() {
    //System.out.println("hood goal: " + goal);
    hood.moveToPos(goal);
    goal = drive.calculateHoodAngle();
    System.out.println(drive.getPose());


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