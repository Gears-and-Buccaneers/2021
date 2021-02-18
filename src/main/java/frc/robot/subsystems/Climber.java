/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.Constants.ClimberConstants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  private static TalonSRX winchMotor = new TalonSRX(ClimberConstants.kClimberWinch);
  private static TalonSRX hookMotor = new TalonSRX(ClimberConstants.kClimberHook);
  private static TalonSRX elevatorMotor = new TalonSRX(ClimberConstants.kClimberElevator);
  private static DoubleSolenoid cliberSol = new DoubleSolenoid(ClimberConstants.kClimberSolenoidPorts[0],ClimberConstants.kClimberSolenoidPorts[1]);

  private boolean climberExtended;
  private double startTime;
  
  /**
   * Creates a new Climber.
   */
  public Climber() {
    climberExtended = false;
    startTime = Timer.getFPGATimestamp();

    winchMotor.configFactoryDefault(); //clears all previous settings
    hookMotor.configFactoryDefault();
    elevatorMotor.configFactoryDefault();

    winchMotor.setNeutralMode(NeutralMode.Brake); //makes sure the motors attempt to stay in the same place once stopped.
    hookMotor.setNeutralMode(NeutralMode.Brake);
    elevatorMotor.setNeutralMode(NeutralMode.Brake);
    elevatorMotor.setInverted(false);

    cliberSol.set(Value.kForward);
  }

  /**
   * @return the climberExtended
   */
  public boolean isClimberExtended() {
    return climberExtended;
  }

  /**
   * extends the climber to a set position
   */
  public void extendClimber() {
    // while(Timer.getFPGATimestamp() - startTime < ClimberConstants.kExtendTimeInSeconds) {
    //   winchMotor.set(ControlMode.PercentOutput, -1);
    // }
    
    elevatorMotor.set(ControlMode.PercentOutput, 1);
    //cliberSol.set(Value.kReverse);
    climberExtended = true;
  }

  public void extendPiston(){
    cliberSol.set(Value.kReverse);
  }

  public void retractPiston(){
    cliberSol.set(Value.kForward);
  }

  public void driveOnBar(double left, double right) {
    double leftInput = -left;
    double rightInput =  right;
    double output = leftInput + rightInput;
    hookMotor.set(ControlMode.PercentOutput, output);
  }

  public void winch(){
    winchMotor.set(ControlMode.PercentOutput, -1);
  }

  public void stopWinch(){
    winchMotor.set(ControlMode.PercentOutput, 0);
  }

  public void reverseWinch(){
    winchMotor.set(ControlMode.PercentOutput, 0.3);
  }

  public void stop(){
    elevatorMotor.set(ControlMode.PercentOutput, 0);
    //cliberSol.set(Value.kForward);
  }
  public void reverse(){
    elevatorMotor.set(ControlMode.PercentOutput, -0.4);
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
