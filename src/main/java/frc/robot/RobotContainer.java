/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDStrip;
import frc.robot.subsystems.Storage;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Limelight;
import frc.robot.commands.AlignWithVision;
import frc.robot.commands.Auto1;
import frc.robot.commands.Auto2;
import frc.robot.commands.AutoPID;
import frc.robot.commands.IndexBalls;
import frc.robot.commands.IndexOrReverse;
import frc.robot.commands.ExhaustBalls;

import static edu.wpi.first.wpilibj.XboxController.Button;

import java.util.List;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final Drivetrain m_drivetrain = new Drivetrain();
  private final Intake m_intake = new Intake();
  private final Shooter m_shooter = new Shooter();
  private final Storage m_storage = new Storage();
  private final Climber m_climber = new Climber();

  private final Limelight m_limelight = new Limelight();

  public final LEDStrip m_ledStrip = new LEDStrip();

  //colors for led strip
  private double[] blue = new double[]{0,0,255};
  private double[] white = new double[]{100,100,100};

  // A chooser for autonomous commands
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  //the sendable auto routines to chose from
  private final CommandBase m_auto1 = new Auto1(m_drivetrain, m_shooter, m_storage, m_intake, 4096);
  private final CommandBase m_auto2 = new Auto2(m_drivetrain);

  // The driver's controller
  public XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
    

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    // Set the default drive command to split-stick arcade drive
    m_drivetrain.setDefaultCommand(
        // A split-stick arcade command, with forward/backward controlled by the left
        // hand, and turning controlled by the right.
        new RunCommand(() -> m_drivetrain
            .arcadeDrive(DriveConstants.kDriveCoefficient * m_driverController.getRawAxis(1),
                         DriveConstants.kTurnCoefficient * m_driverController.getRawAxis(2)), m_drivetrain));

    m_limelight.setDefaultCommand(
      new RunCommand(() -> m_limelight.update(), m_limelight) // makes the limelight update to the smartdashboard constantly                                                           // constantly
    );

    m_storage.setDefaultCommand(
      new IndexOrReverse(m_intake, m_storage, m_driverController.getRawAxis(3))
      //new RunCommand(m_storage::stop, m_storage)
    );

    // Add commands to the autonomous command chooser
    m_chooser.addOption("backwards and shoot", m_auto1);
    m_chooser.addOption("forwards off line", m_auto2);
    // Put the chooser on the dashboard
    Shuffleboard.getTab("Autonomous").add(m_chooser);

  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Pop intake out when the right bumper is pressed.
    new JoystickButton(m_driverController, Button.kBumperRight.value)
      .whenPressed( new SequentialCommandGroup(
          new InstantCommand(m_intake::open, m_intake),
          new InstantCommand(m_intake::runNow, m_intake),
          new IndexBalls(m_storage)
      )
    );

    // bring intake in when the left bumper is pressed
    new JoystickButton(m_driverController, Button.kBumperLeft.value).whenPressed(
      new InstantCommand(m_intake::stopRunning, m_intake).andThen(
      new InstantCommand(m_intake::retract, m_intake),
      new InstantCommand(m_storage::stop, m_storage))
    );

    //feed balls while the sqare is held
    new JoystickButton(m_driverController, 1).whileHeld(
          new InstantCommand(m_shooter::runOpenLoop, m_shooter)
          .andThen(
            new InstantCommand(() -> m_ledStrip.setColor(blue)),
            new InstantCommand(m_shooter::runOpenLoop, m_shooter)
          )
    );

    //stop feeding when square is released
    new JoystickButton(m_driverController, 1).whenReleased(
          new InstantCommand(m_shooter::stopShooter, m_shooter)
          .andThen(
            new InstantCommand(() -> m_ledStrip.setColor(white)),
            new InstantCommand(() -> m_limelight.setVisionMode(0)),
            new InstantCommand(m_storage::stop, m_storage)
          )
    );

    //swap driving controls when triangle is pressed
    new JoystickButton(m_driverController, 4).whenPressed(
      new InstantCommand(() -> m_drivetrain.toggleSwap(), m_drivetrain)
    );

    //push balls away while the left stick is pressed
    new JoystickButton(m_driverController, 11)
      .whenPressed(
        new ParallelCommandGroup(
          new RunCommand(() -> m_intake.reverse((m_driverController.getRawAxis(3)+1)/2), m_intake),
          new RunCommand(() -> m_storage.reverse((m_driverController.getRawAxis(3)+1)/2), m_storage)
        )
    );    

    //run climber lifter when option is pressed
      new JoystickButton(m_driverController, 10).whileHeld(
        new InstantCommand(m_climber::extendClimber, m_climber)
    );
    
    //stop climber lifter when option is pressed
    new JoystickButton(m_driverController, 10).whenReleased(
      new InstantCommand(m_climber::stop, m_climber)
    );

    //reverse climber winch for resetting in the pit while share is held
    new JoystickButton(m_driverController, 9).whileHeld(
      new RunCommand(m_climber::reverseWinch, m_climber)
    );

    //stop reversing climber winch when share is held
    new JoystickButton(m_driverController, 9).whenReleased(
      new InstantCommand(m_climber::stopWinch, m_climber)
    );

    //extend the climber piston when the circle is pressed
    new JoystickButton(m_driverController, 3).whenPressed(
      new InstantCommand(m_climber::extendPiston, m_climber)
    );

    //retract the climber piston and lower the climber arm while the right stick is held
    new JoystickButton(m_driverController, 12).whenPressed(
      new SequentialCommandGroup(
        new InstantCommand(m_climber::retractPiston, m_climber),
        new InstantCommand(m_climber::reverse, m_climber)
      )
    );

    //stop lowering the climber arm when the right stick is released
    new JoystickButton(m_driverController, 12).whenReleased(
      new InstantCommand(m_climber::stop, m_climber)
    );
  
    //right button runs transport forward at full speed
    new POVButton(m_driverController, 90).whenPressed(
      new RunCommand(m_storage::run, m_storage)
    );

    //stops running transport when right dpad button is released
    new POVButton(m_driverController, 90).whenReleased(
      new InstantCommand(m_storage::stop, m_storage)
    );

    //winches the robot when the left dpad button is held
    new POVButton(m_driverController , 270).whileHeld(
      new RunCommand(m_climber::winch, m_climber)
    );

    //stops winching when the left dpad button is released
    new POVButton(m_driverController , 270).whenReleased(
      new InstantCommand(m_climber::stopWinch, m_climber)
    );

    //increases shooter by 5% speed when the top dpad button is pressed
    new POVButton(m_driverController, 0).whenPressed(
      new InstantCommand(m_shooter::increaseShooterSpeed, m_shooter)
    );

    //decreases shooter by 5% speed when the bottom dpad button is pressed
    new POVButton(m_driverController, 180).whenPressed(
      new InstantCommand(m_shooter::decreaseShooterSpeed, m_shooter)
    );
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }

  public RamseteCommand getRamseteCommand(){
    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint =
    new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(DriveConstants.ksVolts,
                                    DriveConstants.kvVoltSecondsPerMeter,
                                    DriveConstants.kaVoltSecondsSquaredPerMeter),
                                    DriveConstants.kDriveKinematics,
                                    10);

    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
                            AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(
            new Translation2d(1, 1),
            new Translation2d(2, -1)
        ),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        // Pass config
        config
    );

    RamseteCommand ramseteCommand = new RamseteCommand(
        exampleTrajectory,
        m_drivetrain::getPose,
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        new SimpleMotorFeedforward(DriveConstants.ksVolts,
                                  DriveConstants.kvVoltSecondsPerMeter,
                                  DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics,
        m_drivetrain::getWheelSpeeds,
        new PIDController(DriveConstants.kPDriveVel, 0, 0),
        new PIDController(DriveConstants.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        m_drivetrain::tankDriveVolts,
        m_drivetrain  
    );

    return ramseteCommand;
  }

}
