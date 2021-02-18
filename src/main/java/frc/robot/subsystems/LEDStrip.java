/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifier.LEDChannel;
import com.ctre.phoenix.CANifier.PWMChannel;

import frc.robot.Constants.CANIfierConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDStrip extends SubsystemBase {
  private final CANifier canifier = new CANifier(CANIfierConstants.kCanifier1);

  double[] _dutyCycleAndPeriods = new double[]{0, 0};

  /**
   * Creates a new LEDStrip.
   */
  public LEDStrip() {
    canifier.configFactoryDefault(CANIfierConstants.kTimeoutMs); //reset to factory default

  }

  @Override
  public void periodic() {
    canifier.getPWMInput(PWMChannel.PWMChannel0, _dutyCycleAndPeriods);
	  SmartDashboard.putNumber("lidar distance", _dutyCycleAndPeriods[1]);
  }

  /**
   * sets the colors of the led strip. takes an array of 3 doubles for R, G, and B
   * @param colors
   */
  public void setColor(double[] colors){
    canifier.setLEDOutput(colors[0], LEDChannel.LEDChannelA);
    canifier.setLEDOutput(colors[1], LEDChannel.LEDChannelB);
    canifier.setLEDOutput(colors[2], LEDChannel.LEDChannelC);
  }

  public void setColor(){
    canifier.setLEDOutput(0, LEDChannel.LEDChannelA);
    canifier.setLEDOutput(255, LEDChannel.LEDChannelB);
    canifier.setLEDOutput(0, LEDChannel.LEDChannelC);
  }
}
