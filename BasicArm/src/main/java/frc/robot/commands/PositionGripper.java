// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class PositionGripper extends CommandBase {
  /** Creates a new PositionGripper. */
  private final Arm m_arm;
  private double m_direction = 1;

  /**
   * Creates a new PositionLift command.
   * 
   * @param arm Arm subsystem 
   * @param direction of travel for the gripper 1 (open) or -1 (close)
   * 
   */
  public PositionGripper(Arm arm, int direction) {
    m_arm = arm;
    m_direction = direction;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Move the Gripper until is reaches its max position
    m_arm.gripper(ArmConstants.SERVO_INCREMENT * m_direction); 
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
