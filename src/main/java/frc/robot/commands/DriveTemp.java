package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

import static frc.robot.RobotContainer.tempDrivebase;
import static frc.robot.RobotContainer.operatorController;

public class DriveTemp extends Command {
    private static final double TRANSLATION_SPEED = 0.05; // meters per tick
    private static final double ROTATION_SPEED = 0.03;    // radians per tick

    public DriveTemp() {
        addRequirements(tempDrivebase); // No subsystems needed since it's just a temp pose
    }

    @Override
    public void initialize() {
        // Nothing to initialize
    }

    @Override
    public void execute() {
        // Get joystick inputs
        double leftY = operatorController.getLeftX(); // strafe left/right
        double leftX = operatorController.getLeftY(); // forward/back
        double rightX = -operatorController.getRightX(); // rotation

        // Compute new pose
        double newX = tempDrivebase.getTempPose().getX() + leftX * TRANSLATION_SPEED;
        double newY = tempDrivebase.getTempPose().getY() + leftY * TRANSLATION_SPEED;
        double newRotation = tempDrivebase.getTempPose().getRotation().getRadians() + rightX * ROTATION_SPEED;

        // Update temp pose
        tempDrivebase.setTempPose(new Pose2d(newX, newY, new Rotation2d(newRotation)));
    }

    @Override
    public boolean isFinished() {
        return false; // runs until interrupted
    }
}
