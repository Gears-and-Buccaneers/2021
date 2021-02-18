/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  /**
   * Creates a new Shooter.
   */
  private static final TalonSRX m_ShooterMaster = new TalonSRX(ShooterConstants.kShooterMaster);
  private static final TalonSRX m_ShooterSlave = new TalonSRX(ShooterConstants.kShooterSlave);

  private double pspeed = 0.85;

  public Shooter() {
    //reset talons to factory defaults just in case of a swap or a setting changed in phoenix tuner
    m_ShooterMaster.configFactoryDefault(ShooterConstants.kTimeoutMs);
    m_ShooterSlave.configFactoryDefault(ShooterConstants.kTimeoutMs);

    //makes sure that the motors coast and do not brake so it causes less stress on the motors because it is a shooter
    m_ShooterMaster.setNeutralMode(NeutralMode.Brake);
    m_ShooterSlave.setNeutralMode(NeutralMode.Brake);

    m_ShooterMaster.setInverted(true);

    //make one side (non-encoder motor) slave and reverse it
    m_ShooterSlave.setInverted(false);
    m_ShooterSlave.follow(m_ShooterMaster);

    //set the sensor for the master shooter to be the mag encoder
    m_ShooterMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, ShooterConstants.kPIDLoopIdx, ShooterConstants.kTimeoutMs);
    m_ShooterMaster.setSensorPhase(false); //ensures that encoder is reading in the same direction as motor

    // Config the peak and nominal outputs AKA max and min of the motor controllers
		m_ShooterMaster.configNominalOutputForward(0, ShooterConstants.kTimeoutMs);
		m_ShooterMaster.configNominalOutputReverse(0, ShooterConstants.kTimeoutMs);
		m_ShooterMaster.configPeakOutputForward(1, ShooterConstants.kTimeoutMs);
		m_ShooterMaster.configPeakOutputReverse(-1,ShooterConstants.kTimeoutMs);
		m_ShooterSlave.configNominalOutputForward(0, ShooterConstants.kTimeoutMs);
		m_ShooterSlave.configNominalOutputReverse(0, ShooterConstants.kTimeoutMs);
		m_ShooterSlave.configPeakOutputForward(1, ShooterConstants.kTimeoutMs);
		m_ShooterSlave.configPeakOutputReverse(-1,ShooterConstants.kTimeoutMs);

    // Configure the Velocity closed loop gains in slot0
		m_ShooterMaster.config_kF(ShooterConstants.kPIDLoopIdx, ShooterConstants.kF, ShooterConstants.kTimeoutMs);
		m_ShooterMaster.config_kP(ShooterConstants.kPIDLoopIdx, ShooterConstants.kP, ShooterConstants.kTimeoutMs);
		m_ShooterMaster.config_kI(ShooterConstants.kPIDLoopIdx, ShooterConstants.kI, ShooterConstants.kTimeoutMs);
    m_ShooterMaster.config_kD(ShooterConstants.kPIDLoopIdx, ShooterConstants.kD, ShooterConstants.kTimeoutMs);
    
    m_ShooterMaster.set(ControlMode.PercentOutput, 0);

    SmartDashboard.putNumber("shooter speed", pspeed);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("shooter speed units per 100ms", (m_ShooterMaster.getSelectedSensorVelocity()));
  }

  public void runShooterPID(){
    m_ShooterMaster.set(ControlMode.Velocity, ShooterConstants.targetVelocity_UnitsPer100ms);
  }

  public void decreaseShooterSpeed(){
    if(SmartDashboard.getNumber("shooter speed", 1)<=0.7){
      SmartDashboard.putNumber("shooter speed", 0.7);
    }
    else{
      SmartDashboard.putNumber("shooter speed", pspeed -= 0.05);
    }

  }

  public void increaseShooterSpeed(){
    if(SmartDashboard.getNumber("shooter speed", 1)>1){
      SmartDashboard.putNumber("shooter speed", 1);
    }
    else {
      SmartDashboard.putNumber("shooter speed", pspeed += 0.05);
    }
  }

  public void setShooterSpeed(double speed){
    pspeed = speed;
  }

  public void runOpenLoop(){
    m_ShooterMaster.set(ControlMode.PercentOutput, SmartDashboard.getNumber("shooter speed", 0.86));
    //SmartDashboard.putNumber("shooter speed in rpm", (m_ShooterMaster.getSelectedSensorVelocity()/4096));
  }

  public void stopShooter(){
    m_ShooterMaster.set(ControlMode.PercentOutput, 0);
  }

  public boolean isShooterAtSpeed(){
    return ((m_ShooterMaster.getSelectedSensorVelocity()) > 3800);
  }
}
