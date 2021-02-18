/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.Constants.ShooterConstants;

public class AlignWithVision extends CommandBase {
  private static Drivetrain m_drivetrain;
  private static Limelight m_limelight = new Limelight();

  private boolean m_LimelightHasValidTarget = false;
  private double m_LimelightDriveCommand = 0.0;
  private double m_LimelightSteerCommand = 0.0;

  // These numbers must be tuned for your Robot!  Be careful!
  private double STEER_K = ShooterConstants.STEER_K;                    // how hard to turn toward the target
  private double DRIVE_K = ShooterConstants.DRIVE_K;                    // how hard to drive fwd toward the target
  private double DESIRED_TARGET_AREA = ShooterConstants.DESIRED_TARGET_AREA;        // Area of the target when the robot reaches the wall
  private double MAX_DRIVE = ShooterConstants.MAX_DRIVE;                   // Simple speed limit so we don't drive too fast
  


  /**
   * Creates a new AlignWithVision.
   */
  public AlignWithVision(Drivetrain drivetrain, Limelight limelight) {
    m_drivetrain = drivetrain;
    m_limelight = limelight;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drivetrain, m_limelight);

    SmartDashboard.putNumber("Steering KP", STEER_K);  
    SmartDashboard.putNumber("Desired TA", DESIRED_TARGET_AREA);
    SmartDashboard.putNumber("Driving KP", DRIVE_K);
    SmartDashboard.putNumber("Max speed", MAX_DRIVE);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    STEER_K = SmartDashboard.getNumber("Steering KP", 0.0);
    DESIRED_TARGET_AREA = SmartDashboard.getNumber("min TA", 0.0);
    DRIVE_K = SmartDashboard.getNumber("Driving KP", 0.0);
    MAX_DRIVE = SmartDashboard.getNumber("Max Speed", 0.0);

    m_limelight.setVisionMode(1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_LimelightHasValidTarget) {
        m_drivetrain.arcadeDrive(m_LimelightDriveCommand,m_LimelightSteerCommand);
    }
    else {
      m_drivetrain.arcadeDrive(0.0,0.0);
    }

    SmartDashboard.putNumber("ta", m_limelight.getTA());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.arcadeDrive(0, 0);
    m_limelight.setVisionMode(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_limelight.getTX()<1 && m_limelight.getTX()>-1){
      return true;
    }

    return false;
  }

  public void Update_Limelight_Tracking()
  {
        double tv = m_limelight.getTV();
        double tx = m_limelight.getTX();
        double ty = m_limelight.getTY();
        double ta = m_limelight.getTA();
    
        if (tv < 1.0)
        {
          m_LimelightHasValidTarget = false;
          m_LimelightDriveCommand = 0.0;
          m_LimelightSteerCommand = 0.0;
          return;
        }

        m_LimelightHasValidTarget = true;

        // Start with proportional steering
        double steer_cmd = tx * STEER_K;
        m_LimelightSteerCommand = steer_cmd;

        // try to drive forward until the target area reaches our desired area
        double drive_cmd = (DESIRED_TARGET_AREA - ta) * DRIVE_K;
        //double drive_cmd = 0; //no driving, just turning

        // don't let the robot drive too fast into the goal
        if (drive_cmd > MAX_DRIVE)
        {
          drive_cmd = MAX_DRIVE;
        }
        m_LimelightDriveCommand = drive_cmd;
  }
}
