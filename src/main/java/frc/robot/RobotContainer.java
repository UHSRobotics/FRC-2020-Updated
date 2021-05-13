
package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.DualShockController.Button;
import frc.robot.commands.*;
import frc.robot.commands.pidcommand.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.pidcontroller.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // Subsystem setup
  private final FlywheelSubsystem m_flywheelSubsystem = new FlywheelSubsystem();
  private final LiftSubsystem m_liftSubsystem = new LiftSubsystem();
  private final DigitalInput m_magSwitch = new DigitalInput(3);
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  // private final DropIntakeSubsystem m_DropIntakeSubsystem = new
  // DropIntakeSubsystem();
  private final ServoSubsystem m_servoSubsystem = new ServoSubsystem();
  private final VisionSubsystem m_visionSubsystem = new VisionSubsystem();
  private final HopperSubsystem m_hopper = new HopperSubsystem();
  private final ColorSubsystem m_colorSubsystem = new ColorSubsystem();
  private final SpinSubsystem m_spinSubsystem = new SpinSubsystem();
  private final LiftPID m_liftPID = new LiftPID(m_liftSubsystem);
  private final RotPID m_rotPID = new RotPID(m_driveSubsystem);

  private final ManualShootingCommand m_manualShooting = new ManualShootingCommand(m_flywheelSubsystem, m_hopper,
      m_rotPID, m_driveSubsystem, m_visionSubsystem, -1);
  private final Turning180Command m_turning180 = new Turning180Command(m_rotPID, m_driveSubsystem);
  

  SendableChooser<Command> m_chooser = new SendableChooser<>();
  DualShockController m_driverController = new DualShockController(0);
  DualShockController m_subsystemController = new DualShockController(1);
  
  private final ArcadeDrive m_defaultDrive = new ArcadeDrive(m_driveSubsystem,
  () -> m_driverController.getYMapped(Hand.kLeft), () -> m_driverController.getXMapped(Hand.kRight));
  
    // Autonomous setup
    // private final Command m_autoCommand = 
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_driverController.initMapping(OIConstants.kDriverControllerCurvature);
    configureButtonBindings();
    m_flywheelSubsystem
        .setDefaultCommand(new FlywheelCommand(m_flywheelSubsystem, () -> m_subsystemController.getCrossButton()));
    m_liftSubsystem.setDefaultCommand(
        new LiftCommand(m_liftSubsystem, m_liftPID, () -> m_subsystemController.getBumper(Hand.kLeft),
            () -> m_subsystemController.getBumper(Hand.kRight), m_servoSubsystem));
    m_IntakeSubsystem
        .setDefaultCommand(new IntakeCommand(m_IntakeSubsystem, () -> m_subsystemController.getYMapped(Hand.kLeft)));
    m_driveSubsystem.setDefaultCommand(m_defaultDrive);
    m_hopper.setDefaultCommand(new HopperCommand(m_hopper, () -> m_subsystemController.getYMapped(Hand.kRight),() -> m_subsystemController.getRawButton(Button.kStickRight.value)));
    // m_chooser.setDefaultOption("target", m_autonPlaceholder);
    // m_chooser.addOption("simple drive", m_autoCommand);
    // Shuffleboard.getTab("Autonomous").add(m_chooser);

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // new JoystickButton(m_driverController, Button.kTrig.value)
    // .whileHeld(new IntakeCommand(m_IntakeSubsystem, () ->
    // m_driverController.getTrigButton()));

    // For debugging only, delete before competition
    // new JoystickButton(m_driverController, Button.kBumperLeft.value)
    // .whileHeld(new DistancePIDCommand(m_driveSubsystem, -200));

    // TODO: enable this after making sure rotation PID works
    new JoystickButton(m_driverController, Button.kBumperLeft.value).whenPressed(() -> {
      m_defaultDrive.toggleInvert();
    }).whileHeld(m_turning180).whenReleased(()->{
      m_turning180.stop();
    });

    // new JoystickButton(m_subsystemController, Button.kRect.value)
    // .whenPressed(new VisionDistancePIDCommand(m_driveSubsystem,
    // m_visionSubsystem));

    // new JoystickButton(m_subsystemController, Button.kBumperLeft.value)
    // .whileHeld(new DropIntakeCommand(m_DropIntakeSubsystem,
    // () -> m_subsystemController.getBumperPressed(Hand.kLeft),
    // () -> m_subsystemController.getBumperPressed(Hand.kRight) ));

    // @return 0-1-2-3=blue-green-red-yellow; -1: match revolution

    new JoystickButton(m_subsystemController, Button.kDisk.value)
        .whenPressed(new InstantCommand(m_servoSubsystem::toggle, m_servoSubsystem));

    new JoystickButton(m_driverController, Button.kBumperRight.value).whileHeld(m_manualShooting).whenReleased(() -> {
      m_manualShooting.stop();
    });

    /*
     * new JoystickButton(m_driverController, Button.kBumperRight.value)
     * .whenPressed(new SpinCommand(m_colorSubsystem, m_spinSubsystem, -1)); new
     * JoystickButton(m_driverController, Button.kRect.value) .whenPressed(new
     * SpinCommand(m_colorSubsystem, m_spinSubsystem, 0)); new
     * JoystickButton(m_driverController, Button.kCross.value) .whenPressed(new
     * SpinCommand(m_colorSubsystem, m_spinSubsystem, 1)); new
     * JoystickButton(m_driverController, Button.kDisk.value) .whenPressed(new
     * SpinCommand(m_colorSubsystem, m_spinSubsystem, 2)); new
     * JoystickButton(m_driverController, Button.kTrig.value) .whenPressed(new
     * SpinCommand(m_colorSubsystem, m_spinSubsystem, 3));
     */
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * 
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new Auton(m_flywheelSubsystem, m_hopper, m_rotPID, m_driveSubsystem, m_visionSubsystem);
  }
}