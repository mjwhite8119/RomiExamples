// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TurnToTarget extends PIDCommand {
  private static Drivetrain m_drive;
  private final Vision m_vision;

  /** 
   * Creates a new TurnToTarget command. 
  */
  public TurnToTarget(Drivetrain drive, Vision vision) {
    super(
        // The controller that the command will use
        new PIDController(VisionConstants.kGainsTurn.kP, 0, 0),
        // This should return the measurement
        vision::getYaw,
        // This should return the setpoint (can also be a constant)
        0,
        // This uses the output
        output -> {
          // Use the output here
          drive.arcadeDrive(0, output);
        },
        drive
        );
      m_drive = drive;
      m_vision = vision;
  }

  public void initialize() {
    super.initialize();
    // Make sure everything starts from zero
    m_drive.arcadeDrive(0, 0);
    SmartDashboard.putBoolean("Command Finished", false);
    System.out.println("TurnToTarget Entered");
    // m_vision.resetFilter();
  }

  public void execute() {
    super.execute();
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
