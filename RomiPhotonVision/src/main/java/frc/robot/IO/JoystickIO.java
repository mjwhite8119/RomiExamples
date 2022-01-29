// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.IO;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/** Add your docs here. */
public class JoystickIO {
  private XboxController m_controller;

  public JoystickIO(XboxController controller) {
    m_controller = controller;
  }

  public double xAxisSpeed() {
    return m_controller.getRawAxis(1);
  }

  public double zAxisRotate() {
    if (m_controller.getClass() == XboxController.class) {
      return m_controller.getRawAxis(4);
    } else {
      return m_controller.getRawAxis(2);
    }       
  }

  public double xAxisBoostSpeed() {
    // Default to PS3
    double boostValue = mapRange(-1, 1, 0, 1, m_controller.getRawAxis(3));
    double mainAxisPercent = 0.75;

    if (m_controller.getClass() == XboxController.class) {
      boostValue = m_controller.getRawAxis(2);    
    }
    return xAxisSpeed() * (mainAxisPercent + (1-mainAxisPercent) * boostValue);
  }

  public double zAxisBoostRotate() {
    // Default to PS3
    double boostValue = mapRange(-1, 1, 0, 1, m_controller.getRawAxis(3));
    double mainAxisPercent = 0.6;

    if (m_controller.getClass() == XboxController.class) {
      boostValue = m_controller.getRawAxis(3);    
    }
    return xAxisSpeed() * (mainAxisPercent + (1-mainAxisPercent) * boostValue);
  }

  public Button panLeft() {
    return new JoystickButton(m_controller, XboxController.Button.kX.value);
  }

  public Button panRight() {
    return new JoystickButton(m_controller, XboxController.Button.kB.value);
  }

  public Button tiltUp() {
    return new JoystickButton(m_controller, XboxController.Button.kY.value);
  }

  public Button tiltDown() {
    return new JoystickButton(m_controller, XboxController.Button.kA.value);
  }

  /**
   * Utility function to remap a range of values
   * 
   * @param a1 Low end of range to map from 
   * @param a2 High end of range to map from
   * @param b1 Low end of range to map to
   * @param b2 High end of range to map to
   * @param s  The value that you want to map
   * @return The remapped value
   */
  public double mapRange(double a1, double a2, double b1, double b2, double s){
    return b1 + ((s - a1)*(b2 - b1))/(a2 - a1);
  }

}
