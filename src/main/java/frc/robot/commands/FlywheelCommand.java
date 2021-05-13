/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.pidcontroller.FlywheelPID;

/**
 * A command to drive the robot with joystick input (passed in as
 * {@link DoubleSupplier}s). Written explicitly for pedagogical purposes -
 * actual code should inline a command this simple with
 * {@link edu.wpi.first.wpilibj2.command.RunCommand}.
 */
public class FlywheelCommand extends CommandBase {
  private final FlywheelSubsystem m_neoFw;
  // private final HopperSubsystem m_hopper = new HopperSubsystem();
  // private final FlywheelPID m_flywheelPID;
  // private final HopperSubsystem m_hopper;
  private final BooleanSupplier m_fullPow;// , m_mag;

  public FlywheelCommand(FlywheelSubsystem subsystem, BooleanSupplier fullPow) {
    m_neoFw = subsystem;
    // m_flywheelPID = pid;
    // m_hopper = hopper;
    m_fullPow = fullPow;
    // m_mag = mag;
    addRequirements(m_neoFw);
  }

  @Override
  public void execute() {
    if (m_fullPow.getAsBoolean()) {
      // m_neoFw.setSpeed(1);
      // m_hopper.switchON(0.8);
      m_neoFw.setPIDTarget(4000);
    } else {
      // m_neoFw.setSpeed(0);
      // m_hopper.switchOFF();
      m_neoFw.setPIDTarget(0);
    }

    // pid test
    /*
     * if (m_fullPow.getAsBoolean()) { m_flywheelPID.setSetpoint(700); if
     * (!m_flywheelPID.isEnabled()) { m_flywheelPID.enable(); } } else { if
     * (m_flywheelPID.isEnabled()) { m_flywheelPID.disable(); } m_neoFw.setSpeed(0);
     * } if (m_neoFw.getRate() == 700) { m_hopper.switchON(1); }
     */

  }
}
