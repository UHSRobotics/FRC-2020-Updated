/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Ports;

public class IntakeSubsystem extends SubsystemBase {
    private final TalonSRX m_motor = new TalonSRX(Ports.intake);

    private final ShuffleboardTab tab = Shuffleboard.getTab("Scoring");
    private NetworkTableEntry speedEntry;
    private double speedMultiplier = 1;

    public IntakeSubsystem() {
        // set motors to coast
        m_motor.setNeutralMode(NeutralMode.Coast);
        m_motor.setInverted(true);
    }

    public void move(double vel) {
        m_motor.set(ControlMode.PercentOutput, vel * speedMultiplier);
    }

    public void setSpeedMultiplier(double speed, boolean updateNT) {
        if (0 <= speed && speed <= 2) {
            speedMultiplier = speed;
            if (updateNT) {
                System.out.println("NT update (intake)");
                speedEntry.setDouble(speedMultiplier);
            }
        } else {
            System.out.println("NT update (intake)");
            speedEntry.setDouble(speedMultiplier);
        }
    }

    @Override
    public void periodic() {
        if (speedEntry == null) {
            speedEntry = tab.addPersistent("Intake Speed Multiplier", 1).getEntry();
            System.out.println("NT update (intake)");
        }
        setSpeedMultiplier(speedEntry.getDouble(0.3), false);
    }
}