/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class DropIntakeSubsystem extends SubsystemBase {
  private final TalonSRX m_motor;

  private final ShuffleboardTab tab = Shuffleboard.getTab("Scoring");
  private NetworkTableEntry speedEntry;
  private final double speedMultiplier = 0.5;
  
  public DropIntakeSubsystem() {
     // set motors to coast
     m_motor = new TalonSRX(1);
     m_motor.setNeutralMode(NeutralMode.Coast);
  }

  public void dropIntake() {
     System.out.println("fml");
    m_motor.set(ControlMode.PercentOutput, speedMultiplier);
  }

  public void raiseIntake() {
    // System.out.print("ffffffml");
    m_motor.set(ControlMode.PercentOutput, speedMultiplier);
  }

  public void stopIntake() {
    m_motor.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    if (speedEntry == null) {
      speedEntry = tab.add("Drop Intake Speed", 1).getEntry();
      System.out.println("Added Single Speed Multiplier NT entry");
  }
  // dropIntake(); 
 }
}
