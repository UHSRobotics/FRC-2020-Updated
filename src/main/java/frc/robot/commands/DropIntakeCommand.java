/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DropIntakeSubsystem;

public class DropIntakeCommand extends CommandBase {
  private final DropIntakeSubsystem m_dropIntake;
  private final BooleanSupplier m_data;
  private final BooleanSupplier m_dataT;

  public DropIntakeCommand(DropIntakeSubsystem m_dropIntake, BooleanSupplier m_data, BooleanSupplier m_dataT) {
    this.m_dropIntake = m_dropIntake;
    this.m_data = m_data;
    this.m_dataT = m_dataT;
    addRequirements(m_dropIntake);
  }


// Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    if (m_data.getAsBoolean()) {
      if (m_dataT.getAsBoolean()) {
        m_dropIntake.raiseIntake();
      }
      m_dropIntake.dropIntake();
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    m_dropIntake.stopIntake();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run

}
