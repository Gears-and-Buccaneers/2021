/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Storage;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class Auto1 extends SequentialCommandGroup {
  
  /**
   * Creates a new Auto1.
   */
  public Auto1(Drivetrain drivetrain, Shooter shooter, Storage storage, Intake intake, double distance) {
    addCommands(
      new InstantCommand(intake::open, intake),
      new WaitCommand(0.6),
      new InstantCommand(intake::runNow, intake),
      new WaitCommand(0.5),
      new InstantCommand(shooter::runOpenLoop, shooter),
      new InstantCommand(() -> shooter.setShooterSpeed(0.85), shooter),
      new WaitUntilCommand(shooter::isShooterAtSpeed),
      new WaitCommand(1.5),
      new InstantCommand(storage::run, storage),
      new WaitCommand(2),
      new InstantCommand(intake::stopRunning, intake),
      new InstantCommand(intake::retract, intake),
      new WaitCommand(1),
      new InstantCommand(intake::runNow, intake),
      new WaitCommand(3),
      new InstantCommand(shooter::stopShooter, shooter),
      new InstantCommand(storage::stop, storage),
      new InstantCommand(intake::stopRunning),
      new DriveStraight(drivetrain, 2, true)
    );
  }
}
