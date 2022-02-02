// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.CameraMount;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class CloseToTarget extends CommandBase {

  private final Drivetrain m_drive;
  private final Vision m_vision;
  private final CameraMount m_camera_mount;
  private final double m_distance;
  
  private double m_currentRange;

  PIDController m_forwardController = new PIDController(VisionConstants.kGainsForward.kP, 0, 0);
        
  PIDController m_turnController = new PIDController(VisionConstants.kGainsTurn.kP, 0, 0);
  
  // Moving average filter used to smooth out control outputs
  private MedianFilter m_filter = new MedianFilter(10);

  /** 
   * Constructor
   * Closes in on a target using a camera that is mounted on pan/tilt
   * servos. As the robot moves towards the target the camera will pan
   * back to the foreward facing position.
   *
   * @param drive The drivetrain subsystem on which this command will run
   * @param vision The RomiCamera subsystem
   * @param cameraMount The servo activated pan/tilt camera mount.
   * @param distance The distance that the robot should end up from the target.
  */
  public CloseToTarget(Drivetrain drive, 
                       Vision vision, 
                       CameraMount cameraMount,
                       double distance) 
  {
    m_drive = drive;
    m_distance = distance;
    m_camera_mount = cameraMount;
    m_vision = vision;
    addRequirements(drive, cameraMount);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.arcadeDrive(0, 0);
    m_camera_mount.getPan().setDefaultAngle(98);
    m_camera_mount.getTilt().setDefaultAngle(94);
    SmartDashboard.putBoolean("Command Finished", false);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // First calculate range
    m_currentRange = m_vision.getRange();
    double feedForward = 0.3;
    // Use this range as the measurement we give to the PID controller.
    // -1.0 required to ensure positive PID controller effort _increases_ yaw
    var forwardSpeed = -m_forwardController.calculate(m_currentRange, m_distance);

    double panAngle = m_camera_mount.getPanAngle();
    SmartDashboard.putNumber("Camera Pan angle", panAngle);

    var rotationSpeed = -m_turnController.calculate(m_vision.getYaw(), 0);

    m_drive.arcadeDrive(m_filter.calculate(forwardSpeed) + feedForward, rotationSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("Command Finished", true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_currentRange <= m_distance || m_vision.lostTarget();
  }
}
