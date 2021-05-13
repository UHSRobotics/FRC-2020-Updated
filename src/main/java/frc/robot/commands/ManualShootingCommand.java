/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.Constants.HopperConstants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.pidcontroller.RotPID;

public class ManualShootingCommand extends CommandBase {
  /**
   * Creates a new ManualShootingCommand.
   */
  FlywheelSubsystem m_flywheel;
  HopperSubsystem m_hopper;
  BooleanSupplier m_fullPow;
  RotPID m_rotPID;
  VisionSubsystem m_vision;
  DriveSubsystem m_drive;
  private boolean started = false, firstTime = false;
  private double flywheelRPM = FlywheelConstants.targetRPM;
  double timeoutCounter= -10000;
  /**
   * should be put into a "whileHeld"
   */
  public ManualShootingCommand(FlywheelSubsystem flywheel, HopperSubsystem hopper, RotPID rotPID, DriveSubsystem drive,
      VisionSubsystem vision, double timeout) {
    m_drive = drive;
    m_flywheel = flywheel;
    m_hopper = hopper;
    m_rotPID = rotPID;
    m_vision = vision;
    addRequirements(m_flywheel);
    addRequirements(m_hopper);
    addRequirements(rotPID);
    if(timeout>0){
      timeoutCounter = timeout/20;
    }
    started = false;
  }

  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(timeoutCounter>=0) timeoutCounter--;
    SmartDashboard.putBoolean("manual shooting enabled", started);
    if (m_vision.working()) {
      if (!started) {
        firstTime = true;
        m_drive.disableManual();
        m_rotPID.setGoalRelative((m_vision.getHorizontalAngle() / Math.PI) * -180.0);
        if (!m_rotPID.isEnabled())
          m_rotPID.enable();
        updateFlywheelSpeed();
        started = true;
      }
      if (m_rotPID.getController().atSetpoint()) {
        if (firstTime) {
          firstTime = false;
          updateFlywheelSpeed();
        }
        SmartDashboard.putBoolean("rot pid settled", true);

        if (m_flywheel.atSetPoint()) {
          m_hopper.setPIDTarget(HopperConstants.targetRPM);
        } else {
          m_hopper.setPIDTarget(0);
        }
      } else {
        SmartDashboard.putBoolean("rot pid settled", false);
        if (!m_rotPID.isEnabled())
          m_rotPID.enable();
        firstTime = true;
      }
    } else {
      if (firstTime) {
        m_drive.enableManual();
        if (m_rotPID.isEnabled())
          m_rotPID.disable();

        m_flywheel.setPIDTarget(flywheelRPM);
        firstTime = false;
      }
      if (m_flywheel.atSetPoint()) {
        m_hopper.setPIDTarget(HopperConstants.targetRPM);
      } else {
        m_hopper.setPIDTarget(0);
      }
    }
  }

  private void updateFlywheelSpeed() {
    if (m_vision.getX() < 3) {
      m_flywheel.setPIDTarget(FlywheelConstants.targetRPM + 600);
    } else if (m_vision.getX() < 4.5) {
      m_flywheel.setPIDTarget(FlywheelConstants.targetRPM + 200);
    } else if (m_vision.getX() < 6) {
      m_flywheel.setPIDTarget(FlywheelConstants.targetRPM);
    } else if (m_vision.getX() < 7) {
      m_flywheel.setPIDTarget(FlywheelConstants.targetRPM + 300);
    } else {
      m_flywheel.setPIDTarget(FlywheelConstants.targetRPM + 750);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    stop();
  }

  public void stop() {
    m_drive.enableManual();
    m_flywheel.setPIDTarget(0);
    if (m_rotPID.isEnabled())
      m_rotPID.disable();
    m_hopper.setPIDTarget(0);
    started = false;
    SmartDashboard.putBoolean("manual shooting enabled", started);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (timeoutCounter < 0 && timeoutCounter > -5000);
  }
}
