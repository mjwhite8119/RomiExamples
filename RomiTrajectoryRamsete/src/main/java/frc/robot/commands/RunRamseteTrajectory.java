// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.ControlConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RunRamseteTrajectory extends RamseteCommand {
  Drivetrain m_drivetrain;
  Trajectory m_trajectory;

  /** Creates a new RunRamseteTrajectory. */
  public RunRamseteTrajectory(Drivetrain drivetrain, Trajectory trajectory) {
    super(
      trajectory,
      drivetrain::getPose,
      new RamseteController(ControlConstants.kRamseteB, ControlConstants.kRamseteZeta),
      DrivetrainConstants.kFeedForward,
      DrivetrainConstants.kDriveKinematics,
      drivetrain::getWheelSpeeds,
      new PIDController(DrivetrainConstants.kPDriveVelLeft, 0, 0),
      new PIDController(DrivetrainConstants.kPDriveVelRight, 0, 0),
      drivetrain::tankDriveVolts,
      drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    m_drivetrain = drivetrain;
    m_trajectory = trajectory;
  }

  public void initialize() {
    super.initialize();
    m_drivetrain.resetOdometry(m_trajectory.getInitialPose());   
  }

}
