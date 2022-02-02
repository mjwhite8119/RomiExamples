// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class MoveToTarget extends CommandBase {
  private final Drivetrain m_drive;
  private final Vision m_vision;
  private final double m_distance;
  private double m_currentRange;

  PIDController m_forwardController = new PIDController(VisionConstants.kGainsForward.kP, 0, 0);

  ProfiledPIDController m_profiledController = 
          new ProfiledPIDController(
            // The PID gains and motion profile constraints
            DriveConstants.kGainsDriveVel.kP,
            DriveConstants.kGainsDriveVel.kI,
            DriveConstants.kGainsDriveVel.kD,
            DriveConstants.kTrapezoidProfileConstraints);
        
  PIDController m_turnController = new PIDController(VisionConstants.kGainsTurn.kP, 0, 0);
  
  // Moving average filter used to smooth out control outputs
  private MedianFilter m_filter = new MedianFilter(10);
  
  /** 
   * Constructor
   * Moves towards a target that is sent from the vision system. 
   * 
   * @param distance The distance that the robot should end up from the target.
   * @param drive The drivetrain subsystem on which this command will run
   * @param vision The RomiCamera subsystem
   */
  public MoveToTarget(Drivetrain drive, Vision vision, double distance) {
    m_drive = drive;
    m_distance = distance;
    m_vision = vision;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.arcadeDrive(0, 0);
    SmartDashboard.putBoolean("Command Finished", false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // First calculate range
    m_currentRange = m_vision.getRange();
    double feedForward = 0.3;
    // Use this range as the measurement we give to the PID controller.
    // -1.0 required to ensure positive PID controller effort _increases_ yaw
    var forwardSpeed = -m_forwardController.calculate(m_currentRange, m_distance);
    // var forwardSpeed = -m_profiledController.calculate(m_currentRange, 
    //                                                   new TrapezoidProfile.State(m_distance,0));

    var rotationSpeed = -m_turnController.calculate(m_vision.getYaw(), 0);

    m_drive.arcadeDrive(m_filter.calculate(forwardSpeed) + feedForward, rotationSpeed);
    // m_drive.arcadeDrive(m_filter.calculate(forwardSpeed), rotationSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("Command Finished", true);
    System.out.println(m_currentRange);
    System.out.println(m_distance);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_currentRange <= m_distance || m_vision.lostTarget();
  }
}
