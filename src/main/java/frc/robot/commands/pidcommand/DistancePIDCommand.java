package frc.robot.commands.pidcommand;

import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PhysicalMeasurements;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.*;

public class DistancePIDCommand extends ProfiledPIDCommand {
    /**
     * @param goal in cm
     */
    public DistancePIDCommand(DriveSubsystem drive, double goal) {
        super(new ProfiledPIDController(DriveConstants.KpDist, DriveConstants.KiDist, DriveConstants.KdDist,
                new Constraints(DriveConstants.velLimit, DriveConstants.accelLimit)),
                // Close loop on heading
                drive::getAvgEncCM,
                // Set reference to target
                drive.getAvgEncCM() + goal,
                // Pipe output to turn robot
                (output, setpoint) -> drive.arcadeDriveAuton(-output, 0),
                // Require the drive
                drive);
        m_controller.setTolerance(10);

        System.out.println("goal is " + goal / (PhysicalMeasurements.wheelDiam * Math.PI) * 4096);

        // Set the controller to be continuous (because it is an angle controller)
        // getController().enableContinuousInput(-180, 180);
        // Set the controller tolerance - the delta tolerance ensures the robot is
        // stationary at the
        // setpoint before it is considered as having reached the reference
    }

    @Override
    public boolean isFinished() {
        // End when the controller is at the reference.
        return getController().atGoal();
    }
}