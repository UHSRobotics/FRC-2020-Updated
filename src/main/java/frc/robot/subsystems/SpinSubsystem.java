package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class SpinSubsystem extends SubsystemBase {
    private final TalonSRX m_spinner = new TalonSRX(Constants.SpinCons.spinner);
    private final ShuffleboardTab tab = Shuffleboard.getTab("Spinner");
    private NetworkTableEntry spinnerEntry;

    public void spin(double pow) {
        if(spinnerEntry==null){
            spinnerEntry = tab.addPersistent("Spinner Speed", 1).getEntry();
            System.out.println("Added spinner NT entry");
        }
        spinnerEntry.setDouble(pow);
        m_spinner.set(ControlMode.PercentOutput, pow);
    }
}