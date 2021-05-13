package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.subsystems.pidcontroller.DriveRotationPID;
// import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class ArcadeDrive extends CommandBase {
  private final DriveSubsystem m_drive;
  private final DoubleSupplier m_pow;
  private final DoubleSupplier m_turn;
  private boolean invert = false;
  
  // private final DriveRotationPID m_turnPID = new DriveRotationPID();

  public ArcadeDrive(DriveSubsystem subsystem, DoubleSupplier powerSupplier, DoubleSupplier turnSupplier) {
    m_drive = subsystem;
    m_pow = powerSupplier;
    m_turn = turnSupplier;
    // m_reverse = reverseSupplier;
    addRequirements(m_drive);
    // addRequirements(m_turnPID);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putBoolean("Inverted", invert);
    if(!invert)
      m_drive.arcadeDrive(m_pow.getAsDouble(), m_turn.getAsDouble());
    else
      m_drive.arcadeDrive(-m_pow.getAsDouble(), m_turn.getAsDouble());
  }

  public void toggleInvert(){
    invert= !invert;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.arcadeDriveAuton(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
