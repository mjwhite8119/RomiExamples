// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.CameraMount;
import frc.robot.subsystems.Vision;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PanToTarget extends PIDCommand {
  private static CameraMount m_camera_mount;
  private final Vision m_vision;
  private double m_last_error = 0;

  /** 
   * Creates a new TurnToTarget command. 
  */
  public PanToTarget(CameraMount cameraMount, Vision vision) {
    super(
        // The controller that the command will use
        new PIDController(VisionConstants.kGainsPan.kP, 0, 0),
        // This should return the measurement
        vision::getYaw,
        // This should return the setpoint (can also be a constant)
        0,
        // This uses the output
        output -> {
          // Use the output here
          cameraMount.pan(output);
        },
        cameraMount
        );
      m_camera_mount = cameraMount;
      m_vision = vision;
  }

  public void initialize() {
    super.initialize();
    SmartDashboard.putBoolean("Command Finished", false);
    System.out.println("TurnToTarget Entered");
  }

  public void execute() {
    super.execute();
    // If we lost the target then keep going in the last direction
    // to attempt to reaquire it.  
    if (m_vision.hasTargets() == false) {
      if (m_last_error > 0) {
        m_camera_mount.pan(1.0);
      } else {
        m_camera_mount.pan(-1.0);
      }  
    }
    else 
    {
      m_last_error = getController().getPositionError();
    }
    
    SmartDashboard.putNumber("Error", getController().getPositionError());    
  }

  @Override
  public void end(boolean interrupted) {
      SmartDashboard.putBoolean("Command Finished", true);
      System.out.println("TurnToTarget Exited");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
    // return m_vision.hasTargets() == false;
  }
}
