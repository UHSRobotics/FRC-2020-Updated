/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.Constants.Ports;
import edu.wpi.first.wpilibj.smartdashboard.*;

public class FlywheelSubsystem extends SubsystemBase {
  /**
   * Creates a new NeoFwSubsystem.
   */
  private final CANSparkMax m_motor = new CANSparkMax(Ports.flywheel, MotorType.kBrushless);
  private final CANSparkMax m_motor2 = new CANSparkMax(Ports.flywheelInvert, MotorType.kBrushless);
  private final CANPIDController c = m_motor.getPIDController();
  private final CANPIDController c2 = m_motor2.getPIDController();

  // private final CANSparkMax m_motorInverted = new CANSparkMax(1,
  // MotorType.kBrushless);
  private final ShuffleboardTab tab = Shuffleboard.getTab("Scoring");
  private NetworkTableEntry speedEntry, velocityEntry, pidStatusEntry;
  private double speedMultiplier = 1, targetVelocity = 1.0;

  public FlywheelSubsystem() {
    m_motor.setIdleMode(IdleMode.kCoast);
    m_motor2.setIdleMode(IdleMode.kCoast);
    m_motor2.setInverted(true);
    // m_motorInverted.setInverted(true);
    c.setP(FlywheelConstants.Kp);
    c.setI(FlywheelConstants.Ki);
    c.setD(FlywheelConstants.Kd);
    c.setFF(FlywheelConstants.Kff);
    c.setOutputRange(-1, 1);

    c2.setP(FlywheelConstants.Kp);
    c2.setI(FlywheelConstants.Ki);
    c2.setD(FlywheelConstants.Kd);
    c2.setFF(FlywheelConstants.Kff);
    c2.setOutputRange(-1, 1);
  }

  public void setSpeed(double p) {
    p *= speedMultiplier;
    // System.out.println(p);
    m_motor.set(p);
    m_motor2.set(p);
  }

  /**
   * 
   * @return velocity in rpm
   */
  public double getRate() {
    return m_motor.getEncoder().getVelocity();
  }

  public void setSpeedMultiplier(double speed, boolean updateNT) {
    if (0 <= speed && speed <= 2) {
      speedMultiplier = speed;
      if (updateNT) {
        System.out.println("Putted Single Speed Multiplier NT entry");
        speedEntry.setDouble(speedMultiplier);
      }
    } else {
      System.out.println("Putted Single Speed Multiplier NT entry");
      speedEntry.setDouble(speedMultiplier);
    }
  }

  public void setPIDTarget(double t) {
    t*=speedMultiplier;
    if (targetVelocity == 0 && t != 0) {
      c.setP(FlywheelConstants.Kp);
      c.setI(FlywheelConstants.Ki);
      c.setD(FlywheelConstants.Kd);
      c.setFF(FlywheelConstants.Kff);

      c2.setP(FlywheelConstants.Kp);
      c2.setI(FlywheelConstants.Ki);
      c2.setD(FlywheelConstants.Kd);
      c2.setFF(FlywheelConstants.Kff);
    }else if(targetVelocity!=0 && t==0){
      c.setP(FlywheelConstants.Kp*FlywheelConstants.slowK);
      c.setI(FlywheelConstants.Ki *FlywheelConstants.slowK);
      c.setD(FlywheelConstants.Kd *FlywheelConstants.slowK);
      c.setFF(FlywheelConstants.Kff *FlywheelConstants.slowK);

      c2.setP(FlywheelConstants.Kp *FlywheelConstants.slowK);
      c2.setI(FlywheelConstants.Ki *FlywheelConstants.slowK);
      c2.setD(FlywheelConstants.Kd *FlywheelConstants.slowK);
      c2.setFF(FlywheelConstants.Kff *FlywheelConstants.slowK);
    }
    targetVelocity = t;
    c.setReference(targetVelocity, ControlType.kVelocity);
    c2.setReference(targetVelocity, ControlType.kVelocity);
  }

  public boolean atSetPoint() {
    return Math.abs(getRate() - targetVelocity) < FlywheelConstants.tolRPM;
  }

  @Override
  public void periodic() {
    if (speedEntry == null) {
      speedEntry = tab.addPersistent("Neo Speed Multiplier", 1).getEntry();
      System.out.println("Added Single Speed Multiplier NT entry");
    }
    setSpeedMultiplier(speedEntry.getDouble(1.0), false);
    if (velocityEntry == null) {
      velocityEntry = tab.add("Velocity (RPM)", 1).getEntry();
      System.out.println("added velocity entry");
    }
    velocityEntry.setDouble(getRate());

    if (pidStatusEntry == null) {
      pidStatusEntry = tab.add("PID At Setpoint", false).getEntry();
      System.out.println("added flywheel pid status entry");
    }
    pidStatusEntry.setBoolean(atSetPoint());
  }
}
