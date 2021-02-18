/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveStraight extends CommandBase {
  private Drivetrain m_drivetrain;
  private double time;
  private double timeinit;
  private boolean direction;
  
  /**
   * Creates a new DriveStraight.
   */
  public DriveStraight(Drivetrain drivetrain, double time, boolean direction) {
    m_drivetrain = drivetrain;
    this.time = time;
    this.direction = direction;
    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timeinit = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    while(Timer.getFPGATimestamp()-timeinit < time){
      if(direction){
        m_drivetrain.drive(-0.5, -0.544);
      }
      else{
        m_drivetrain.drive(0.544, 0.5);
      }
    }
    m_drivetrain.drive(0, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.drive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
