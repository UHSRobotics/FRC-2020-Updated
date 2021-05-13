package frc.robot.subsystems.pidcontroller;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.Constants.LiftConstants;
import frc.robot.Constants.Ports;
import frc.robot.subsystems.FlywheelSubsystem;
// import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LiftSubsystem;

public class FlywheelPID extends PIDSubsystem {
    private final FlywheelSubsystem m_flywheel;

    public FlywheelPID() {
        super(new PIDController(FlywheelConstants.Kp, FlywheelConstants.Ki, FlywheelConstants.Kd), 0);
        m_flywheel = new FlywheelSubsystem();
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        m_flywheel.setSpeed(output);
    }

    @Override
    protected double getMeasurement() {
        return m_flywheel.getRate();
    }

}