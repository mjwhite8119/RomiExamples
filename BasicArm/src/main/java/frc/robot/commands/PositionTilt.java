// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class PositionTilt extends CommandBase {
  private final Arm m_arm;
  private double m_direction = 1;
  
  /**  Creates a new PositionTilt command. 
   *  
   * @param arm Arm subsystem
   * @param direction of travel for the arm 1 or -1
   * 
  */
  public PositionTilt(Arm arm, int direction) {
    m_arm = arm;
    m_direction = direction;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Starting max tilt..");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Move the tilt UP or DOWN until is reaches its max position
    m_arm.tilt(ArmConstants.SERVO_INCREMENT * m_direction);          
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_direction == 1) {
      if (m_arm.tiltAtMax()) {
        System.out.println("FINISHED UP Tilt=" + m_arm.getTiltPos());
        return true;
      }  
    } else {
      if (m_direction == -1) {
        if (m_arm.tiltAtMin()) {
          System.out.println("FINISHED DOWN Tilt=" + m_arm.getTiltPos());
          return true;
        } 
      }      
    }  
    return false;
  }
}
