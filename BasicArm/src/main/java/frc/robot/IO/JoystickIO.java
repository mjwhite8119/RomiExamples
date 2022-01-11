// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.IO;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/** Add your docs here. */
public class JoystickIO {
  private XboxController m_controller;

  public JoystickIO(XboxController controller) {
    m_controller = controller;
  }

  public Button moveTiltUp() {
    return new JoystickButton(m_controller, XboxController.Button.kX.value);
  }

  public Button moveTiltDown() {
    return new JoystickButton(m_controller, XboxController.Button.kB.value);
  }

  public Button moveLiftUp() {
    return new JoystickButton(m_controller, XboxController.Button.kY.value);
  }

  public Button moveLiftDown() {
    return new JoystickButton(m_controller, XboxController.Button.kA.value);
  }

  public Button moveArmFullUp() {
    return new JoystickButton(m_controller, XboxController.Button.kBack.value);
  }

  public Button moveArmFullDown() {
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
