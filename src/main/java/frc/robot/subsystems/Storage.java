/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.BallStorageConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Storage extends SubsystemBase {
  private static final TalonSRX topMotor = new TalonSRX(BallStorageConstants.kStorageMotorMaster);
  private static final TalonSRX bottomMotor = new TalonSRX(BallStorageConstants.kStorageMotorSlave);

  private static final AnalogInput entranceSensor = new AnalogInput(BallStorageConstants.entranceSensor);
  private static final AnalogInput exitSensor = new AnalogInput(BallStorageConstants.exitSensor);

  private int numBalls = 0;

  public Storage() {
    topMotor.configFactoryDefault();
    bottomMotor.configFactoryDefault();

    topMotor.setNeutralMode(NeutralMode.Brake);
    bottomMotor.setNeutralMode(NeutralMode.Brake);

    topMotor.setInverted(false);

    bottomMotor.setInverted(true);
    bottomMotor.follow(topMotor);
  }

  public void run(){
    topMotor.set(ControlMode.PercentOutput, 1);
  }

  public void stop(){
    topMotor.set(ControlMode.PercentOutput, 0);
  }

  public void reverse(){
    topMotor.set(ControlMode.PercentOutput, -1);
  }
  public void reverse(double input){
    topMotor.set(ControlMode.PercentOutput, -input);
  }


  public void countBalls(){
    isPresentOnEntry();
    isPresentOnExit();
  }

  public boolean isPresentOnEntry(){
    if(entranceSensor.getVoltage() <= BallStorageConstants.minRecognizeVoltage){
      SmartDashboard.putNumber("number of balls in storage", numBalls++);
      return true;
    }
    else{
      return false;
    }
  }

  public boolean isPresentOnExit(){
    if(exitSensor.getVoltage() <= BallStorageConstants.minRecognizeVoltage){
      //SmartDashboard.putNumber("number of balls in storage", numBalls);
      return true;
    }
    else{
      return false;
    }
  }

  public int getNumBalls(){
    return numBalls;
  }

  /**
   * @param numBalls the numBalls to set
   */
  public void setNumBalls(int numBalls) {
    this.numBalls = numBalls;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
