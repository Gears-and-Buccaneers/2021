/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Storage;
import frc.robot.subsystems.Intake;

public class IndexOrReverse extends CommandBase {
  private Intake m_intake = new Intake();
  private Storage m_storage = new Storage();

  private double triggerVal;
  
  /**
   * Creates a new IndexOrReverse.
   */
  public IndexOrReverse(Intake intake, Storage storage, double triggerValue) {
    m_intake = intake;
    m_storage  = storage;

    addRequirements(intake, storage);

    triggerVal = triggerValue;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(triggerVal > 0){
      m_storage.reverse((triggerVal+1)/2);
      m_intake.reverse((triggerVal+1)/2);
    }
    else{
      new IndexBalls(m_storage);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
