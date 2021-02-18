/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  private static NetworkTable m_table = NetworkTableInstance.getDefault().getTable("limelight"); //gets the network table where the limelight streams vision tracking data.
  //x location of the target
  private static NetworkTableEntry tx = m_table.getEntry("tx");
  //y location of the target
  private static NetworkTableEntry ty = m_table.getEntry("ty");
  //area of the target
  private static NetworkTableEntry ta = m_table.getEntry("ta");
  //does the limelight have a target
  private static NetworkTableEntry tv = m_table.getEntry("tv");

  /**
   * Creates a new Limelight.
   */
  public Limelight() {
    setVisionMode(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getTX(){
    return tx.getDouble(0.0);
  }

  public double getTY(){
    return ty.getDouble(0.0);
  }

  public double getTA(){
    return ta.getDouble(0.0);
  }

  public double getTV(){
    return tv.getDouble(0.0);
  }

  public boolean isTargetAvalible(){
    return tv.getBoolean(false);
  }

  public void setVisionMode(int visionMode){
    m_table.getEntry("pipeline").setNumber(visionMode); //sets which vision pipeline we are using (should only be one)
  }

  public static String getVisionMode(){
    return  m_table.getEntry("getpipe").getString("0");
  }

  public void update() {
    SmartDashboard.putNumber("limelight x", getTX());
    SmartDashboard.putNumber("limelight y", getTY());
    SmartDashboard.putBoolean("limelight has target", isTargetAvalible());
    SmartDashboard.putString("limelight mode", getVisionMode());
  }
}
