// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.Turret;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.DriveToPoseML;
import frc.robot.commands.DriveToPoseStraight;
import frc.robot.commands.ClimberCommands.ClimbDefault;
import frc.robot.commands.ClimberCommands.GoToPosition;
import frc.robot.commands.FlywheelCommands.TurnToSpeed;
import frc.robot.commands.HoodCommands.HoodDefault;
import frc.robot.commands.HoodCommands.RotateToPosition;
import frc.robot.commands.IntakeBeltCommands.IntakeDefault;
import frc.robot.commands.IntakeBeltCommands.IntakeDown;
import frc.robot.commands.IntakeBeltCommands.IntakeStuff;
import frc.robot.commands.SpindexerCommands.SpinDefault;
import frc.robot.commands.SpindexerCommands.spin2;
import frc.robot.commands.TurretCommands.HomeTurret;
import frc.robot.commands.TurretCommands.TurnToHub;
import frc.robot.commands.TurretCommands.TurnToPosition;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.IntakeBelt;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  public static Drive drive;

  public static Vision vision;

    public static Spindexer spindexer;

    public static Climber climber;

    public static Flywheel flywheel;

    public static Hood hood;

    public static IntakeBelt intakeBelt;

    public static Turret turret;


  //  public static IntakeBelt intakeBelt;
  // Controller
  public static CommandXboxController controller = new CommandXboxController(0);
  public static CommandXboxController opController = new CommandXboxController(1);


  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        vision =
            new Vision(
                drive::addVisionMeasurement,
               // new VisionIOLimelight("limelight-rturret", drive::getRotation),
                new VisionIOLimelight("limelight-lturret", drive::getRotation),
                new VisionIOLimelight("limelight-rturret", drive::getRotation));
        spindexer = new Spindexer();
        flywheel = new Flywheel();
        climber = new Climber();
        hood = new Hood();
        intakeBelt = new IntakeBelt();
        turret = new Turret();
       // intakeBelt = new IntakeBelt();
        // The ModuleIOTalonFXS implementation provides an example implementation for
        // TalonFXS controller connected to a CANdi with a PWM encoder. The
        // implementations
        // of ModuleIOTalonFX, ModuleIOTalonFXS, and ModuleIOSpark (from the Spark
        // swerve
        // template) can be freely intermixed to support alternative hardware
        // arrangements.
        // Please see the AdvantageKit template documentation for more information:
        // https://docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template#custom-module-implementations
        //
        // drive =
        // new Drive(
        // new GyroIOPigeon2(),
        // new ModuleIOTalonFXS(TunerConstants.FrontLeft),
        // new ModuleIOTalonFXS(TunerConstants.FrontRight),
        // new ModuleIOTalonFXS(TunerConstants.BackLeft),
        // new ModuleIOTalonFXS(TunerConstants.BackRight));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});

        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
    configureNamedCommands();
  }

  public void configureNamedCommands() {
    NamedCommands.registerCommand("Climb L1", new GoToPosition(climber, 10));
    NamedCommands.registerCommand("Lower Climb", new GoToPosition(climber, 0));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    intakeBelt.setDefaultCommand(new IntakeDefault(intakeBelt));
    spindexer.setDefaultCommand(new SpinDefault(spindexer));
    hood.setDefaultCommand(new HoodDefault(hood));
    climber.setDefaultCommand(new ClimbDefault(climber));


    //opController.back().toggleOnTrue(new RotateToPosition(hood, 0.3));
    opController.leftStick().toggleOnTrue(new spin2(spindexer, false));
    opController.pov(0).toggleOnTrue(new IntakeStuff(intakeBelt, true));
    opController.pov(180).toggleOnTrue(new IntakeStuff(intakeBelt, false));
    //opController.pov(90).toggleOnTrue(new TurnToSpeed(flywheel, -44));

    //ACTUAL FLYWHEEL/HOOD COMMANDS
    opController.leftBumper().toggleOnTrue(new RotateToPosition(hood));
    opController.rightBumper().toggleOnTrue(new TurnToSpeed(flywheel));
   
    //TESTING FLYWHEEL/HOOD COMMANDS
    //opController.leftBumper().toggleOnTrue(new RotateToPosition(drive, hood));
    //opController.rightBumper().toggleOnTrue(new TurnToSpeed(flywheel, -39.75));    
    opController.rightStick().toggleOnTrue(new spin2(spindexer, true));

   
   
    // opController.pov(270).toggleOnTrue(new RotateToPosition(hood, 0.05));

   opController.rightStick().toggleOnTrue(new GoToPosition(climber, 0.5));

    opController.a().onTrue(new HomeTurret(turret));
    opController.x().onTrue(new TurnToPosition(turret, -0.19));
    opController.b().onTrue(new TurnToPosition(turret, 0.19));
    opController.y().onTrue(new TurnToHub(turret));




    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Lock to 0° when A button is held
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> Rotation2d.kZero));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    controller
        .start()
        .toggleOnTrue(
            new DriveToPoseStraight(
                drive, new Pose2d(new Translation2d(4.6269-3.7338, 4.03), new Rotation2d(Units.degreesToRadians(0)))));
    controller
        .rightTrigger()
        .toggleOnTrue(
            new DriveToPoseStraight(
                drive, new Pose2d(new Translation2d(3, 4), new Rotation2d(0))));

    controller
        .back()
        .toggleOnTrue(
            new DriveToPoseStraight(
                drive, new Pose2d(new Translation2d(2, 2), new Rotation2d(Units.degreesToRadians(0)))));

    // opController
    //     .start()
    //     .toggleOnTrue(
    //         new DriveToPoseStraight(
    //             drive, new Pose2d(new Translation2d(1.08, 4.6), new Rotation2d(Units.degreesToRadians(0)))));

    //opController.a().toggleOnTrue(new DriveToPoseML(drive));
    // Reset gyro to 0° when B button is pressed
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

   // controller.leftBumper().toggleOnTrue(new GoToPosition(climber, 10));
   // controller.leftBumper().toggleOnFalse(new GoToPosition(climber, 0));

   // controller.y().toggleOnTrue()
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
