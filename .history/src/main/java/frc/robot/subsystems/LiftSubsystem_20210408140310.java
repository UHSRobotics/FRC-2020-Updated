/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.LiftConstants;
import frc.robot.Constants.Ports;

public class LiftSubsystem extends SubsystemBase {
  private final TalonSRX m_liftMotor = new TalonSRX(Constants.Ports.lift);
  private final TalonSRX m_follow = new TalonSRX(Constants.Ports.liftFollow);
  // private final
  private double speedMultiplier = 0.25;
  private NetworkTableEntry speedEntry, encoderEntry, velEntry, powerEntry;
  private final ShuffleboardTab tab = Shuffleboard.getTab("Lift");
  private static boolean init = false;
  private static int initialEncoder = 0;
  // private final Encoder m_encoder;

  private double targetSpeed, curSpeed;

  public LiftSubsystem() {
    // m_liftMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
    // m_encoder = new Encoder(Ports.liftEncoderA, Ports.liftEncoderB, false,
    // EncodingType.k4X);
    m_liftMotor.setNeutralMode(NeutralMode.Coast);
    m_follow.setNeutralMode(NeutralMode.Coast);
    m_follow.follow(m_liftMotor);
    m_liftMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    m_liftMotor.configClearPositionOnLimitF(true, 10);
    m_liftMotor.configClearPositionOnLimitR(true, 10);
    initialEncoder = getEncoder();

  }

  public int getEncoder() {
    return -1 * m_liftMotor.getSelectedSensorPosition() - initialEncoder;
  }

  //in RPS revolution per second
  public double getEncoderVelocity() {
    return m_liftMotor.getSelectedSensorVelocity() * -10.0 / 4096.0;
  }

  public void setSpeed(double s) {
    if ((getEncoder() < LiftConstants.liftLowerBound && s < 0)
        || (getEncoder() > LiftConstants.liftUpperBound && s > 0)) {
      curSpeed = targetSpeed = 0;
      m_liftMotor.set(ControlMode.PercentOutput, 0);
      return;
    }

    s=Math.abs(s)<=1?s:Math.signum(s) * 1;

    targetSpeed = curSpeed = s;
    s *= speedMultiplier; 

    if(powerEntry == null){
      powerEntry = tab.add("Lift Power",1).getEntry();
      System.out.println("Added Power NT entry");
    }
    powerEntry.setDouble(s);

    m_liftMotor.set(ControlMode.PercentOutput, s);
  }

  public void setVelTarget(double s) {
    targetSpeed = s;
  }

  public void setSpeedMultiplier(double speed) {
    setSpeedMultiplier(speed, true);
  }

  public void setSpeedMultiplier(double speed, boolean updateNT) {
    if (0 <= speed && speed <= 1) {
      speedMultiplier = speed;
      if (updateNT) {
        System.out.println("Lift Speed Multiplier update");
        speedEntry.setDouble(speedMultiplier);
      }
    } else {
      System.out.println("Lift Speed Multiplier update");
      speedEntry.setDouble(speedMultiplier);
    }
  }

  @Override
  public void periodic() {
    if (speedEntry == null) {
      speedEntry = tab.addPersistent("Lift Speed Multiplier", 0.25).getEntry();
      System.out.println("Added Lift Speed Multiplier NT entry");
    }
    if (encoderEntry == null) {
      encoderEntry = tab.add("Encoder Reading", 1).getEntry();
      System.out.println("Added Lift Encoder Reading NT entry");
    }
    setSpeedMultiplier(speedEntry.getDouble(0.25), false);
    encoderEntry.setDouble(getEncoder());

    if(velEntry == null){
      velEntry = tab.add("Lift Velocity", 1).getEntry();
      System.out.println("Added Lift Velocity Reading NT entry");
    }
    velEntry.setDouble(getEncoderVelocity());
    
    if (getEncoder() < LiftConstants.liftLowerBound) {
      targetSpeed = targetSpeed < 0 ? 0 : targetSpeed;
      curSpeed = curSpeed < 0 ? 0 : curSpeed;
    } else if (getEncoder() > LiftConstants.liftUpperBound) {
      targetSpeed = targetSpeed > 0 ? 0 : targetSpeed;
      curSpeed = curSpeed > 0 ? 0 : curSpeed;
    }
    curSpeed += Math.min(LiftConstants.liftAccelLimit, Math.abs(targetSpeed - curSpeed))
        * Math.signum(targetSpeed - curSpeed);
    m_liftMotor.set(ControlMode.PercentOutput, curSpeed * speedMultiplier);
  }

  public void initialize() {
    init = true;
  }

  public boolean getInit() {
    return init;
  }
}