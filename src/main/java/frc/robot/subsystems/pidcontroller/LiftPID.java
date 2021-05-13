package frc.robot.subsystems.pidcontroller;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants.LiftConstants;
import frc.robot.Constants.Ports;
// import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LiftSubsystem;

public class LiftPID extends PIDSubsystem {
    private final LiftSubsystem m_lift;
    private double lastSetpoint = 0;
    public LiftPID(LiftSubsystem lift) {
        super(new PIDController(LiftConstants.Kp, LiftConstants.Ki, LiftConstants.Kd), 0);
        m_lift = lift;
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        m_lift.setVelTarget(output);
    }

    @Override
    protected double getMeasurement() {
        return m_lift.getEncoderVelocity();
    }

    public void setTarget(double setpoint){
        if(setpoint!=lastSetpoint)getController().reset();
        setSetpoint(setpoint);
        lastSetpoint = setpoint;
    }

}