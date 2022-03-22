// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ExtIOConstants;

public class Bumper extends SubsystemBase {
  
  private final DigitalInput m_bumper_button = new DigitalInput(ExtIOConstants.DIO0_PORT);

  /** 
   * Constructor
   * Creates a new Bumper. The button is attached to an external IO port.
   * You can choose various actions if the bumper runs into anything.
   * See RobotContainer for an example.
   * 
   */
  public Bumper() {}

  /** Gets if the button connected to extIO0 is pressed. */
  public boolean getBumperPressed() {
    return m_bumper_button.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
