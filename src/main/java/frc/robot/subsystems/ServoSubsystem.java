/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Ports;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class ServoSubsystem extends SubsystemBase {
    private final Servo m_switch = new Servo(Ports.servo);
    public static boolean toggleOn = false;
    private ShuffleboardTab servoTab = Shuffleboard.getTab("Lift");
    private NetworkTableEntry servoEntry;
    private static double angle = 0;

    public void toggle() {
        toggleOn = !toggleOn;
        angle = m_switch.getAngle();
        System.out.print(angle);
    }

    @Override
    public void periodic() {
        if (servoEntry == null)
            servoEntry = servoTab.add("Servo Status", "Climb Init").getEntry();
        if (toggleOn) {
            servoEntry.setBoolean(true);
            m_switch.setAngle(200); // ratchet engaged

        } else {
            servoEntry.setBoolean(false);
            m_switch.setAngle(40); // ratchet disengaged
        }
    }

    /**
     * @return true = ratchet engaged
     */
    public boolean getToggle() {
        return toggleOn;
    }

}
