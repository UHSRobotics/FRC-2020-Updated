package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;

public class AutonomousSequence extends CommandBase {
    private final DriveSubsystem m_drive;
    private final FlywheelSubsystem m_fw;
    private final HopperSubsystem m_hop;
    boolean finished = false;

    public AutonomousSequence(final DriveSubsystem drive, FlywheelSubsystem fw, HopperSubsystem hop) {
        m_drive = drive;
        m_fw = fw;
        m_hop = hop;
        addRequirements(m_drive);
        addRequirements(m_fw);
        addRequirements(m_hop);
    }

    @Override
    public void execute(){
        m_drive.arcadeDriveAuton(-0.3, 0);
        Timer.delay(0.5);
        m_drive.arcadeDriveAuton(0, 0);
        m_fw.setSpeed(1);
        Timer.delay(2);
        m_hop.switchON(1);
        Timer.delay(5);
        m_fw.setSpeed(0);
        m_hop.switchOFF();
        m_drive.arcadeDriveAuton(-0.3, 0);
        Timer.delay(1);
        m_drive.arcadeDriveAuton(0, 0);
        finished = true;



    }

    @Override
    public boolean isFinished() {
        return finished;
    }

    @Override
    public void end(boolean b) {
    }
}