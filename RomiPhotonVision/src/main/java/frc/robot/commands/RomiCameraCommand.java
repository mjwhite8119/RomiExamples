// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Vision;

public class RomiCameraCommand extends CommandBase {
  private final Vision m_vision;

  /** 
   * Constructor
   * This command can be run as a default command to display various
   * derived values from the camera
   * 
   * @param vision The RomiCamera subsystem
   */
  public RomiCameraCommand(Vision vision) {
    m_vision = vision;
    addRequirements(vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("Range Target Height", m_vision.getTargetHeight());
    SmartDashboard.putNumber("Range Camera Pitch", m_vision.getCameraPitch());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Range Target Pitch", 
                        Math.tan(m_vision.getCameraPitch() + m_vision.getPitchRadians()));
    SmartDashboard.putNumber("Range Pitch", m_vision.getPitchRadians());
    SmartDashboard.putNumber("Range", m_vision.getRange());
    SmartDashboard.putBoolean("Lost Target", m_vision.lostTarget());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
