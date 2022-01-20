// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.oi;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/** Add your docs here. */
public class DriverOI {
  private XboxController m_controller;

  public DriverOI(XboxController controller) {
    m_controller = controller;
  }

  public Button resetOdometry() {
    return new JoystickButton(m_controller, XboxController.Button.kStart.value);
  }

  public Button useGripper() {
    return new Button(() -> {
        return m_controller.getLeftTriggerAxis() > 0.1 && 
               m_controller.getRightTriggerAxis() > 0.1;
    });
  }

  public Button openGripper() {
    return new Button(() -> {
        return m_controller.getLeftTriggerAxis() > 0.1;
    });
  }

  public Button closeGripper() {
    return new Button(() -> {
        return m_controller.getRightTriggerAxis() > 0.1;
    });
  }

}
