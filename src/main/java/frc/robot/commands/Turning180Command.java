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

public class Turning180Command extends CommandBase {
  /**
   * Creates a new Turning180Command.
   */
  RotPID m_rotPID;
  DriveSubsystem m_drive;
  private boolean started = false;

  /**
   * should be put into a "whileHeld"
   */
  public Turning180Command(RotPID rotPID, DriveSubsystem drive) {
    m_drive = drive;
    m_rotPID = rotPID;
    // addRequirements(m_drive);
    addRequirements(rotPID);
    started = false;
  }

  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!started) {
      m_rotPID.setGoalRelative(180.0);
      started = true;
    }
    if (!m_rotPID.isEnabled())
      m_rotPID.enable();
    m_drive.disableManual();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    stop();
  }

  public void stop() {
    m_drive.enableManual();
    if (m_rotPID.isEnabled())
      m_rotPID.disable();
    started = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_rotPID.getController().atSetpoint();
  }
}
