/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Storage;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ExhaustBalls extends CommandBase {
  private static Storage m_storage;
  private static Shooter m_shooter;

  double timeInitial;

  public ExhaustBalls(Storage subsystem, Shooter shooter) {
    m_storage = subsystem;
    m_shooter = shooter;
    addRequirements(m_storage, m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timeInitial = Timer.getFPGATimestamp();

    m_shooter.runOpenLoop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_shooter.isShooterAtSpeed()){
      // for(int i = m_storage.getNumBalls(); i > 0; i--){
      //   while(Timer.getFPGATimestamp() - timeInitial < 0.3){
      //     m_storage.run();  
      //   }
      //   m_storage.stop();
      //   timeInitial = Timer.getFPGATimestamp();
      // }  
      m_storage.run();
    }
    else {
      m_storage.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.stopShooter();
    m_storage.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    while(m_storage.getNumBalls()>0){
      return false;
    }

    return true;
  }
}
