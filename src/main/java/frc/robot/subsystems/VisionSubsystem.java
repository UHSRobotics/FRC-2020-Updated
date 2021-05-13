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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class VisionSubsystem extends SubsystemBase {
  /**
   * Creates a new VisionSubsystem.
   */
  
  private double[] defaultArray;
  // private double scale = 0;
  private NetworkTableEntry targetPose;
  private NetworkTableEntry yaw;
  private final ShuffleboardTab tab = Shuffleboard.getTab("Vision");
  private NetworkTableEntry scaleEntry; 
  private double m_scale=1;
  private int error=0;
  private int cnt = 0;
  private double lastX = 0;
  
  private boolean working = false;

  NetworkTable cameraTable;

  public VisionSubsystem() {
    defaultArray = new double[3];
    defaultArray[0] = 0.0;
    defaultArray[1] = 0.0;
    defaultArray[2] = 0.0;
    NetworkTableInstance table  = NetworkTableInstance.getDefault();
    cameraTable  = table.getTable("chameleon-vision").getSubTable("USB Camera-B4.09.24.1");
    targetPose = cameraTable.getEntry("targetPose");
    yaw = cameraTable.getEntry("yaw");
  }

  public boolean working(){
    return working;
  }

  public double getX(){
    return targetPose.getDoubleArray(defaultArray)[0];
  }

  public double getY(){
    return targetPose.getDoubleArray(defaultArray)[1];
  }
  
  //in radians
  public double getAngle(){
    return targetPose.getDoubleArray(defaultArray)[2];
  }

  public double getZ(){
    return 2.5-Constants.VisionControlConstants.cameraHeight;
  } 
  public double getYaw(){
    return yaw.getDouble(0.0);
  }
  /**
   * 
   * @return in meters
   */
  public double getDistanceFromTarget(){
    double x = getX(), y = getY(), dis = Math.sqrt(x*x*m_scale + y*y*m_scale);
    double mi = Constants.VisionControlConstants.comfortMin, mx = Constants.VisionControlConstants.comfortMax;
    if(dis>mi&&dis<mx) return 0;
    else if(dis<mi){
      return dis-mi;
    }
    else{
      return dis-mx;
    }
  }

  /**
   * @return angle to test if possible to shoot, in radian
   */
  public double getHorizontalAngle(){
    double y = getY(), x = getX();
    // - 0.4 to get the center of the goal, instead of the corner
    return Math.atan(y/x);
  }
  


  public double getRotationDeficit(){
    return -1;
  }

  // public boolean possibleShootingPos(){
  //   if(Math.abs(getHorizontalAngle())>=Constants.VisionControlConstants.innerPortAngleLimit){
  //     // System.out.println("not possible to shoot");
  //     return false;

  //   } 

  //   return true;
  // }
  public void setScale(double scale, boolean updateNT){
    m_scale = scale;
    scaleEntry.setDouble(m_scale);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("vision x",getX());
    SmartDashboard.putNumber("vision horizontal",(getHorizontalAngle()/Math.PI)*180.0);
    
    if(scaleEntry == null)
    scaleEntry = tab.addPersistent("scale", 1).getEntry();
    setScale(scaleEntry.getDouble(1.0), true);

    if(getY()==0.0 && error<=5)
      error++;
    else if(Math.abs(lastX-getX())<0.1 && error<8)
      error+=2;
    else if(Math.abs(lastX-getX())>5)
      error+=Math.abs(lastX-getX())/5 + 6;
    else
      error--;

    lastX = getX();

    if(error>50){
      error=50;
      working = false;
    }
    else if(error>5){
      working = false;
    }else if(error < -5){
      error = -5;
      working = true;
    } 

    SmartDashboard.putBoolean("Vision Working",working);
  }
}
