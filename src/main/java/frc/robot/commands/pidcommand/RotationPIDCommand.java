package frc.robot.commands.pidcommand;

import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.*;

public class RotationPIDCommand extends ProfiledPIDCommand {
    /**
     * @param goal in degrees, clockwise
     */
    public RotationPIDCommand(DriveSubsystem drive, double goal) {
        super(new ProfiledPIDController(DriveConstants.KpRot, DriveConstants.KiRot, DriveConstants.KdRot,
                new Constraints(DriveConstants.velLimit, DriveConstants.accelLimit)),
                // Close loop on heading
                drive::getAngleDegrees,
                // Set reference to target
                drive.getAngleDegrees() + goal,
                // Pipe output to turn robot
                (output, setpoint) -> drive.arcadeDriveAuton(0, output), // TODO: make sure the direction is correct
                // Require the drive
                drive);


        // Set the controller to be continuous (because it is an angle controller)
        getController().enableContinuousInput(-180, 180);
        getController().setTolerance(10);

        // Set the controller tolerance - the delta tolerance ensures the robot is
        // stationary at the
        // setpoint before it is considered as having reached the reference
        // getController().setTolerance(DriveConstants.kTurnToleranceDeg,
        // DriveConstants.kTurnRateToleranceDegPerS);
    }

    

    @Override
    public boolean isFinished() {
        // End when the controller is at the reference.
        return getController().atGoal();
    }
}