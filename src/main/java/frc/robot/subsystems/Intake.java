/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private static final WPI_TalonSRX m_intakeMotor = new WPI_TalonSRX(IntakeConstants.kIntakeMotor);
  private static final DoubleSolenoid m_intakeSol = new DoubleSolenoid(IntakeConstants.kIntakeSolenoidForward, IntakeConstants.kIntakeSolenoidReverse);
  
  /**
   * Creates a new Intake.
   */
  public Intake() {
    m_intakeMotor.configFactoryDefault();

    m_intakeMotor.setNeutralMode(NeutralMode.Brake);

    m_intakeMotor.setInverted(false);

    m_intakeSol.set(Value.kReverse);

    m_intakeMotor.set(ControlMode.PercentOutput, 0);
  }

  public void runNow(){
    m_intakeMotor.set(ControlMode.PercentOutput, 0.45);
  }

  public void runWithAnalog(double num){
    m_intakeMotor.set(ControlMode.PercentOutput, num);
    SmartDashboard.putNumber("thing", num);
  }

  public void reverse(){
    m_intakeMotor.set(ControlMode.PercentOutput, -0.5);
  }
  public void reverse(double power){
    m_intakeMotor.set(ControlMode.PercentOutput, -power);
  }

  public void open(){
    m_intakeSol.set(Value.kForward);
  }

  public void retract(){
    m_intakeSol.set(Value.kReverse);
  }

  public void stopRunning(){
    m_intakeMotor.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
