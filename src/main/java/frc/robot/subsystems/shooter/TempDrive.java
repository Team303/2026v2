package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TempDrive extends SubsystemBase {

    private Pose2d tempRobotPose;

    public TempDrive() {
        tempRobotPose = new Pose2d(0,0, new Rotation2d(0));
    }
    @AutoLogOutput(key = "FRIED/Temp")
    public Pose2d getTempPose() {
        return tempRobotPose;
    }

    public void setTempPose(Pose2d newPose) {
        tempRobotPose = newPose;
    }

    @Override
    public void periodic() {
        //System.out.println(tempRobotPose);
    }
}
