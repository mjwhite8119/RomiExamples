// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ServoConstants;
import frc.robot.subsystems.CameraMount;

public class TiltCamera extends CommandBase {

  private final CameraMount m_camera_mount;
  private final double m_angle;
  
  /** Creates a new TiltCamera. */
  public TiltCamera(CameraMount cameraMount, double angle) {
    m_camera_mount = cameraMount;
    m_angle = angle;
    addRequirements(cameraMount);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_camera_mount.getTiltAngle() > m_angle) {
      m_camera_mount.tilt(-ServoConstants.SERVO_INCREMENT);
    } else {
      m_camera_mount.tilt(ServoConstants.SERVO_INCREMENT);
    }   
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_camera_mount.getTiltAngle() == m_angle;
  }
}
