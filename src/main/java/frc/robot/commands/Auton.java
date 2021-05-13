/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.pidcommand.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.pidcontroller.*;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class Auton extends SequentialCommandGroup {
  /**
   * Creates a new Auton.
   */
  public Auton(FlywheelSubsystem flywheel, HopperSubsystem hopper, RotPID rotPID, DriveSubsystem drive, VisionSubsystem vision){
    // Add your commands in the super() call.  Add the deadline first.
    super(
      new DistancePIDCommand(drive, -200),
      new ManualShootingCommand(flywheel, hopper, rotPID, drive, vision, 8000),
      new Turning180Command(rotPID, drive));
  }
}
