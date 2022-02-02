// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.CameraMountCommand;
import frc.robot.commands.CameraMountLineFollow;
import frc.robot.commands.LineFollow;
import frc.robot.commands.MoveToTarget;
import frc.robot.commands.PanCamera;
import frc.robot.commands.ResetOdometry;
import frc.robot.commands.RomiCameraCommand;
import frc.robot.commands.StopMotors;
import frc.robot.commands.TiltCamera;
import frc.robot.commands.TurnDegrees;
import frc.robot.commands.TurnToTarget;
import frc.robot.oi.DriverOI;
import frc.robot.subsystems.CameraMount;
// import frc.robot.subsystems.Bumper;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.OnBoardIO;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.OnBoardIO.ChannelMode;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.Button;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final OnBoardIO m_onboardIO = new OnBoardIO(ChannelMode.INPUT, ChannelMode.INPUT);

  // Assumes a gamepad plugged into channnel 0
  // private final Joystick m_joystick = new Joystick(0);
  private final XboxController m_joystick = new XboxController(0);
  // private final Bumper m_bumper = new Bumper();
  private final DriverOI m_driverOI = new DriverOI(m_joystick);

  // Add vision components
  private final CameraMount m_camera_mount = new CameraMount();
  private final Vision m_vision = new Vision();

  // Create SmartDashboard chooser for autonomous routines
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();

  // NOTE: The I/O pin functionality of the 5 exposed I/O pins depends on the hardware "overlay"
  // that is specified when launching the wpilib-ws server on the Romi raspberry pi.
  // By default, the following are available (listed in order from inside of the board to outside):
  // - DIO 8 (mapped to Arduino pin 11, closest to the inside of the board)
  // - Analog In 0 (mapped to Analog Channel 6 / Arduino Pin 4)
  // - Analog In 1 (mapped to Analog Channel 2 / Arduino Pin 20)
  // - PWM 2 (mapped to Arduino Pin 21)
  // - PWM 3 (mapped to Arduino Pin 22)
  //
  // Your subsystem configuration should take the overlays into account

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command is arcade drive. This will run unless another command
    // is scheduled over it.
    m_drivetrain.setDefaultCommand(getArcadeDriveCommand());

    // Default command to control the camera mount servos if they are installed
    m_camera_mount.setDefaultCommand( new CameraMountCommand(m_camera_mount, m_driverOI));

    // Default command to run the RomiCamera which gets its data from PhotonVision.
    m_vision.setDefaultCommand(new RomiCameraCommand(m_vision));

    // Example of how to use the onboard IO
    Button onboardButtonA = new Button(m_onboardIO::getButtonAPressed);
    onboardButtonA
        .whenActive(new PrintCommand("Button A Pressed"))
        .whenInactive(new PrintCommand("Button A Released"));

    // Track target in teleop mode
    m_driverOI.trackTargetButton().whileHeld(new TurnToTarget(m_drivetrain, m_vision));    

    // Setup SmartDashboard options
    m_chooser.setDefaultOption("Move to Target", new MoveToTarget(m_drivetrain, m_vision, 0.10));
    m_chooser.addOption("Follow Line", new CameraMountLineFollow(m_drivetrain, m_camera_mount, m_vision));
    m_chooser.addOption("Reset Odometry", new ResetOdometry(m_drivetrain));
    m_chooser.addOption("Tilt Camera", new TiltCamera(m_camera_mount, 78.0));
    m_chooser.addOption("Pan Camera", new PanCamera(m_camera_mount, 98.0));
    m_chooser.addOption("Turn 180 Degrees", new TurnDegrees(-0.4, 180, m_drivetrain));
    SmartDashboard.putData(m_chooser);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }

  /**
   * Use this to pass the teleop command to the main {@link Robot} class.
   *
   * @return the command to run in teleop
   */
  public Command getArcadeDriveCommand() {
    return new ArcadeDrive(
        m_drivetrain, () -> -m_driverOI.xAxisSpeed(), () -> m_driverOI.zAxisRotate()
        // m_drivetrain, () -> -m_driverOI.xAxisBoostSpeed(), () -> m_driverOI.zAxisBoostRotate()
        );
  }
}
