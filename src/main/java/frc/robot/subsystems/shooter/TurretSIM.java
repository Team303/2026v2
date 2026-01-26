package frc.robot.subsystems.shooter;

import static frc.robot.RobotContainer.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretSIM extends SubsystemBase {
    
    private static int wallahi;

    private static final double BLUE_HUB_X = 4.6269;
    private static final double BLUE_HUB_Y = 4.03;
    private static final double RED_HUB_X = 11.91358;
    private static final double RED_HUB_Y = 4.03;

    public TurretSIM() {
        //God Knows

        wallahi = 0;
    }

    @Override
    public void periodic() {
    }

    public double getBlueHubRotate(Pose2d curPose) {
        double xDIFF = BLUE_HUB_X - curPose.getX();
        double yDIFF = BLUE_HUB_Y - curPose.getY();
        double radiansRotate = -Math.atan(yDIFF / xDIFF);

        double finalRadiansRotate = radiansRotate + curPose.getRotation().getRadians();
        double finalAngleRotate = Math.toDegrees(finalRadiansRotate);
        return finalAngleRotate;
    }

    public double getRedHubRotate(Pose2d curPose) {
        double xDIFF = RED_HUB_X - curPose.getX();
        double yDIFF = RED_HUB_Y - curPose.getY(); 
        double radiansRotate = -Math.atan(yDIFF / xDIFF);
        
        double finalRadiansRotate = radiansRotate - normalizeRedRot(curPose.getRotation().getRadians());
        double finalAngleRotate = Math.toDegrees(finalRadiansRotate);

        return finalAngleRotate;
    }

    private double normalizeRedRot(double input) {
        double output = -(input - Math.PI);
        if (output > Math.PI) {
            output -= 2 * Math.PI;
        }
        return output;
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        wallahi++;
        if (wallahi % 5 == 0) {
            Pose2d currentPose = drive.getPose();
            if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
                System.out.println("Turret Rotate: " + getRedHubRotate(currentPose));
            } else {
                System.out.println("Turret Rotate: " + getBlueHubRotate(currentPose));
            }    
        } else if (wallahi > 2_000_000) wallahi = 0;
    }
}
