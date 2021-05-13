package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.pidcommand.DistancePIDCommand;
import frc.robot.commands.pidcommand.RotationPIDCommand;
import frc.robot.subsystems.DriveSubsystem;
public class TargetCommand extends SequentialCommandGroup {
    public TargetCommand(double dis, double rot, DriveSubsystem drive){
        addCommands(new RotationPIDCommand(drive, rot), new DistancePIDCommand(drive, dis));
    }
    
}
