// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.AutonomousDistance;
import frc.robot.commands.AutonomousPIDDistance;
import frc.robot.commands.AutonomousTime;
import frc.robot.commands.DriveDistancePID;
import frc.robot.commands.DriveDistanceProfiled;
import frc.robot.commands.ResetOdometry;
import frc.robot.commands.TurnToAnglePID;
import frc.robot.commands.TurnToAngleProfiled;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.OnBoardIO;
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
  private final Joystick m_controller = new Joystick(0);

  // Create SmartDashboard chooser for autonomous routines
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();

  // Used to get data input from Shuffleboard
  private NetworkTableEntry m_distance;
  private NetworkTableEntry m_distanceP;
  private NetworkTableEntry m_distanceD;

  private NetworkTableEntry m_angle;
  private NetworkTableEntry m_angleP;
  private NetworkTableEntry m_angleD;

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
    configureButtonBindings();
    setupShuffleboard();
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

    // Example of how to use the onboard IO
    Button onboardButtonA = new Button(m_onboardIO::getButtonAPressed);
    onboardButtonA
        .whenActive(new PrintCommand("Button A Pressed"))
        .whenInactive(new PrintCommand("Button A Released"));

    // Setup SmartDashboard options
    m_chooser.setDefaultOption("Profiled Turn Angle PID", new TurnToAngleProfiled(180, m_drivetrain));
    m_chooser.addOption("Drive Distance PID", new DriveDistancePID(1.0, m_drivetrain));
    m_chooser.addOption("Profiled Distance PID", new DriveDistanceProfiled(1.0, m_drivetrain));
    m_chooser.addOption("Turn Degrees PID", new TurnToAnglePID(90, m_drivetrain));
    m_chooser.addOption("Drive Square", new AutonomousPIDDistance(m_drivetrain));   
    m_chooser.addOption("Reset Odometry", new ResetOdometry(m_drivetrain));
    SmartDashboard.putData(m_chooser);
  }

  /**
   * Setup Shuffleboard
   *
   */
  private void setupShuffleboard() {

    // Create a tab for the Drivetrain
    ShuffleboardTab driveTab = Shuffleboard.getTab("Drivetrain");

    // Add the Drivetrain subsystem
    driveTab.add(m_drivetrain)
      .withPosition(6, 0);

    // Add PID tuning parameters
    m_distanceP = driveTab.add("kP", DriveConstants.kPDriveVel)
      .withPosition(3, 1)
      .getEntry();  

    m_distanceD = driveTab.add("kD", DriveConstants.kDDriveVel)
      .withPosition(3, 2)
      .getEntry();  
  
    m_angle = driveTab.add("Heading Angle Degrees", m_drivetrain.getHeading())
      .withPosition(4, 0)
      .getEntry();  

    m_angleP = driveTab.add("anglekP", DriveConstants.kPTurnVel)
      .withPosition(4, 1)
      .getEntry();  

    m_angleD = driveTab.add("anglekD", DriveConstants.kDTurnVel)
      .withPosition(4, 2)
      .getEntry();  
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
        m_drivetrain, () -> -m_controller.getRawAxis(1), () -> m_controller.getRawAxis(2));
  }
}
