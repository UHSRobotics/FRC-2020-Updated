/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.pidcontroller;

import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class RotPID extends ProfiledPIDSubsystem {
  DriveSubsystem m_drive;
  double goal0 = 0;
  public RotPID(DriveSubsystem drive) {
    super(
        // The ProfiledPIDController used by the subsystem
        new ProfiledPIDController(DriveConstants.KpRot, DriveConstants.KiRot, DriveConstants.KdRot,
            new Constraints(DriveConstants.velLimitRot, DriveConstants.accelLimitRot)));
    m_drive = drive;
    m_controller.setTolerance(1);
    // m_controller.enableContinuousInput(-180, 180);
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    m_drive.arcadeDriveAuton(0, output);
  }

  public void setGoalRelative(double goal) {
    if(goal0!=goal)m_controller.reset(getMeasurement()); 
    goal0 = goal;
    setGoal(goal+getMeasurement());
  }

  public double getGoal(){
    return goal0;
  }

  @Override
  public double getMeasurement() {
    return m_drive.getAngleDegrees();
  }
}
