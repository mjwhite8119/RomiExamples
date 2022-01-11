// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Arm;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class PositionArm extends SequentialCommandGroup {
  /** Creates a new PositionArm. 
   *  
   * @param arm Arm subsystem
   * @param direction the arm should move 1 or -1
   * 
  */
  public PositionArm(Arm arm, int direction) {
    addCommands(
      // Start by setting the tilt to max up.
      new PositionTilt(arm, 1),
      new PositionLift(arm, direction));
  }
}
