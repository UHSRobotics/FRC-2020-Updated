/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.HopperConstants;
import frc.robot.Constants.Ports;
import frc.robot.subsystems.HopperSubsystem;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class HopperCommand extends CommandBase {
  /**
   * Creates a new HopperCommand.
   */

  private final HopperSubsystem m_hopper;
  private final DoubleSupplier m_activate;
  private final BooleanSupplier m_isManual;
  // private final AnalogInput m_sensor = new AnalogInput(Ports.ultrasonicSensor);
  // double m_delay = 0.5;
  boolean finished = false;
  double hopperLag = 15;

  public HopperCommand(HopperSubsystem subsystem, DoubleSupplier hopperActivation, BooleanSupplier manualOverride) {
    m_hopper = subsystem;
    m_activate = hopperActivation;
    m_isManual = manualOverride;
    addRequirements(m_hopper);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // SmartDashboard.putNumber("hopper sensor value",m_sensor.getValue());
    // if (!m_isManual.getAsBoolean()) {
      // if (m_sensor.getValue() * 0.125 / 2.54 < HopperConstants.detectionRange) {
      //   hopperLag = 15;
      //   m_hopper.setPIDTarget(HopperConstants.targetRPM);
      // } else {
      //   hopperLag--;
      //   if (hopperLag < 1) {
      //     m_hopper.setPIDTarget(0);
      //   }
      // }
    // } else {
      if (m_activate.getAsDouble() >= 0.5) {
        m_hopper.setPIDTarget(-1 * HopperConstants.targetRPM);
      } else if (m_activate.getAsDouble() <= -0.5) {
        m_hopper.setPIDTarget(HopperConstants.targetRPM);
      } else {
        m_hopper.setPIDTarget(0);
      }
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_hopper.setPIDTarget(0);
    m_hopper.switchOFF();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}