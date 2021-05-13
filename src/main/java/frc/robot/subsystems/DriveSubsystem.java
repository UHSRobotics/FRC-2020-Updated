/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PhysicalMeasurements;
import frc.robot.Constants.PIDConstants;
import frc.robot.Constants.Ports;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {
    private final TalonFX m_leftMotor = new TalonFX(Ports.driveLeft);
    private final TalonFX m_rightMotor = new TalonFX(Ports.driveRight);
    private final TalonFX m_leftFollowMotor = new TalonFX(Ports.driveLeftFollow);
    private final TalonFX m_rightFollowMotor = new TalonFX(Ports.driveRightFollow);

    private final ShuffleboardTab tab = Shuffleboard.getTab("Drive (Falcon 500)");
    private double speedMultiplier = 0.6, turnMultiplier = 0.5;
    private NetworkTableEntry speedEntry, encoderEntry, lpowerEntry, rpowerEntry;

    private double accelLimit = 0.04;
    private double deccelLimit = 0.1;

    private double pow0;
    private boolean manualEnabled = true;

    public DriveSubsystem() {
        pow0 = 0;
        // set motors to Brake
        m_leftMotor.setNeutralMode(NeutralMode.Brake);
        m_rightMotor.setNeutralMode(NeutralMode.Brake);
        m_leftFollowMotor.setNeutralMode(NeutralMode.Brake);
        m_rightFollowMotor.setNeutralMode(NeutralMode.Brake);
        // establish master
        m_leftFollowMotor.follow(m_leftMotor);
        m_rightFollowMotor.follow(m_rightMotor);
        // invert motor
        m_leftMotor.setInverted(true);
        m_rightMotor.setInverted(false);
        m_leftFollowMotor.setInverted(InvertType.FollowMaster);
        m_rightFollowMotor.setInverted(InvertType.FollowMaster);
        // Acceleration

        // reset encoder

        m_leftMotor.getSensorCollection().setIntegratedSensorPosition(0, 0);
        m_rightMotor.getSensorCollection().setIntegratedSensorPosition(0, 0);
        m_leftFollowMotor.getSensorCollection().setIntegratedSensorPosition(0, 0);
        m_rightFollowMotor.getSensorCollection().setIntegratedSensorPosition(0, 0);
        // Integrated PID control
        m_rightMotor.config_kP(PIDConstants.kSlot_Velocit, PIDConstants.kGains_Velocit.kP, PIDConstants.kTimeoutMs);
        m_rightMotor.config_kI(PIDConstants.kSlot_Velocit, PIDConstants.kGains_Velocit.kI, PIDConstants.kTimeoutMs);
        m_rightMotor.config_kD(PIDConstants.kSlot_Velocit, PIDConstants.kGains_Velocit.kD, PIDConstants.kTimeoutMs);
        m_rightMotor.config_kF(PIDConstants.kSlot_Velocit, PIDConstants.kGains_Velocit.kF, PIDConstants.kTimeoutMs);
        m_rightMotor.config_IntegralZone(PIDConstants.kSlot_Velocit, PIDConstants.kGains_Velocit.kIzone,
                PIDConstants.kTimeoutMs);
        m_rightMotor.configClosedLoopPeakOutput(PIDConstants.kSlot_Velocit, PIDConstants.kGains_Velocit.kPeakOutput,
                PIDConstants.kTimeoutMs);
        m_rightMotor.configAllowableClosedloopError(PIDConstants.kSlot_Velocit, 0, PIDConstants.kTimeoutMs);
        // Configure cruise
        m_rightMotor.configMotionCruiseVelocity(15000, PIDConstants.kTimeoutMs);
        m_rightMotor.configMotionAcceleration(6000, PIDConstants.kTimeoutMs);

    }

    public void disableManual(){
        manualEnabled = false;
    }

    public void enableManual(){
        manualEnabled = true;
    }

    /**
     * @param pow  -1 to 1;
     * @param turn positive -> counter-clockwise
     */
    // TODO: make sure positive is actually counter clockwise
    public void arcadeDrive(double pow, double turn) {
        if(!manualEnabled)return;
        
        // jank acceleration control
        pow *= speedMultiplier;
        turn *= turnMultiplier * speedMultiplier;

        double accel = (Math.abs(pow) > Math.abs(pow0)) ? accelLimit : deccelLimit;
        double diff = pow - pow0;

        if (Math.abs(diff) >= accel) {
            pow = pow0 + (diff>0?1:-1) * accel;
        }
        pow0 = pow;

        putPowerEntry(pow - turn, pow + turn);
        m_leftMotor.set(ControlMode.PercentOutput, pow - turn);
        m_rightMotor.set(ControlMode.PercentOutput, pow + turn);
    }

    /**
     * Doesn't go through acceleration limit or speed multiplier
     * 
     * @param pow  -1 to 1;
     * @param turn positive -> counter-clockwise
     */
    public void arcadeDriveAuton(double pow, double turn) {
        putPowerEntry(pow - turn, pow + turn);
        pow = Math.abs(pow)>0.7?0.7 * Math.signum(pow) : pow;
        turn = Math.abs(turn)>0.7?0.7 * Math.signum(turn) : turn;

        m_leftMotor.set(ControlMode.PercentOutput, pow - turn);
        m_rightMotor.set(ControlMode.PercentOutput, pow + turn);
    }

    public void pidDrive(double pos) {
        m_rightMotor.set(ControlMode.Position, pos, DemandType.ArbitraryFeedForward, 0);
        m_leftMotor.follow(m_rightMotor);
    }

    public void motionMagicDrive(double pos) {
        m_rightMotor.set(ControlMode.MotionMagic, pos);
        m_leftMotor.follow(m_rightMotor);
    }

    public void motionMagicTurn(double angle) {
        m_rightMotor.set(ControlMode.MotionMagic, angle);
        m_leftMotor.set(ControlMode.MotionMagic, -angle);
    }

    public void disable() {
        m_rightMotor.set(ControlMode.PercentOutput, 0);
        m_leftMotor.set(ControlMode.PercentOutput, 0);
    }

    public double getAvgEncCM() {
        return encToCm((getEncoderLeft() + getEncoderRight()) / 2.0);
    }

    /**
     * @return positive = clockwise
     */
    public double getAngleDegrees() {
        return (encToCm((getEncoderLeft() - getEncoderRight()) / 2.0) / (PhysicalMeasurements.driveBaseWidth * Math.PI))
        * 360;
    }

    // in centimeters
    public double getEncoderLeft() {
        return -1.0*m_leftMotor.getSelectedSensorPosition();
    }

    // in centimeters
    public double getEncoderRight() {
        return -1.0*m_rightMotor.getSelectedSensorPosition();
    }

    public static double encToCm(double encoderTicks) {
        return (encoderTicks / DriveConstants.ticksPerRev) * (PhysicalMeasurements.wheelDiam * Math.PI);
    }

    public void setSpeedMultiplier(double speed) {
        setSpeedMultiplier(speed, true);
    }

    public void setSpeedMultiplier(double speed, boolean updateNT) {
        if (0 <= speed && speed <= 1) {
            speedMultiplier = speed;
            if (updateNT) {
                System.out.println("Putted Speed Multiplier NT entry");
                speedEntry.setDouble(speedMultiplier);
            }
        } else {
            System.out.println("Putted Speed Multiplier NT entry");
            speedEntry.setDouble(speedMultiplier);
        }
    }

    public void putPowerEntry(double left, double right) {
        if (lpowerEntry == null) {
            lpowerEntry = tab.add("Left Power", 1).getEntry();
            System.out.println("Added Power NT entry");
        }
        if (rpowerEntry == null) {
            rpowerEntry = tab.add("Right Power", 1).getEntry();
            System.out.println("Added Power NT entry");
        }
        lpowerEntry.setDouble(left);
        rpowerEntry.setDouble(right);
    }

    @Override
    public void periodic() {
        if (speedEntry == null) {
            speedEntry = tab.addPersistent("Speed Multiplier", 1).getEntry();
            System.out.println("Added Speed Multiplier NT entry");
        }
        if (encoderEntry == null) {
            encoderEntry = tab.add("Encoder", 1).getEntry();
            System.out.println("Added Encoder NT entry");
        }
        encoderEntry.setDouble(getEncoderRight());
        setSpeedMultiplier(speedEntry.getDouble(1.0), false);
        SmartDashboard.putNumber("angle",getAngleDegrees());

        //TODO: DELETE ME
        SmartDashboard.putNumber("cm",getAvgEncCM());
    }
}
