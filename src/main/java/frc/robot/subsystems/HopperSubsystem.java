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
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HopperConstants;
import frc.robot.Constants.Ports;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class HopperSubsystem extends SubsystemBase {

  /**
   * Creates a new HopperSubsystem.
   */
  private final CANSparkMax m_motor = new CANSparkMax(Ports.hopper, MotorType.kBrushless);
  private final CANPIDController c = m_motor.getPIDController();
  private final ShuffleboardTab tab = Shuffleboard.getTab("Scoring");
  private NetworkTableEntry speedEntry;
  private double speedMultiplier = 1;
  private double targetRPM = 0;

  public HopperSubsystem() {
    m_motor.setInverted(false);
    c.setP(HopperConstants.Kp);
    c.setI(HopperConstants.Ki);
    c.setD(HopperConstants.Kd);
    c.setFF(HopperConstants.Kff);
    c.setOutputRange(-0.3, 0.3);
  }

  public void setPIDTarget(double s) {
    if (targetRPM != s) {
      c.setIAccum(0);
    }
    targetRPM = s;
    c.setReference(s, ControlType.kVelocity);
  }

  public void switchON(double s) {
    s *= speedMultiplier;
    m_motor.set(s);
  }

  public void switchOFF() {
    m_motor.set(0.0);
  }

  public void setSpeedMultiplier(double speed, boolean updateNT) {
    if (0 <= speed && speed <= 2) {
      speedMultiplier = speed;
      if (updateNT) {
        System.out.println("NT update (hopper)");
        speedEntry.setDouble(speedMultiplier);
      }
    } else {
      System.out.println("NT update (hopper)");
      speedEntry.setDouble(speedMultiplier);
    }
  }

  @Override
  public void periodic() {
    if (speedEntry == null) {
      speedEntry = tab.addPersistent("Hopper Speed Multiplier", 1).getEntry();
      System.out.println("NT update (hopper)");
    }
    setSpeedMultiplier(speedEntry.getDouble(0.3), false);
  }
}
