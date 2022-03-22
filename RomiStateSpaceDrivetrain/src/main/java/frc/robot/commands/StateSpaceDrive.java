// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.system.LinearSystemLoop;

public class StateSpaceDrive extends CommandBase {
  /** Creates a new StateSpaceDrive. */
  private final Drivetrain m_drive;
  private final double m_speed;
  private final KalmanFilter<N2, N2, N2> m_observer;
  private final LinearQuadraticRegulator<N2, N2, N2> m_controller;
  private final LinearSystemLoop<N2, N2, N2> m_loop;

  private final TrapezoidProfile.Constraints m_constraints =
      new TrapezoidProfile.Constraints(Constants.DriveConstants.kMaxSpeedMetersPerSecond,
                                       Constants.DriveConstants.kMaxAccelMetersPerSecondSquared);
  private TrapezoidProfile.State m_lastProfiledReference = new TrapezoidProfile.State();

  // Get data from Shuffleboard
  private ShuffleboardTab driveTab = Shuffleboard.getTab("Drivetrain");
  private NetworkTableEntry m_leftVoltage = 
    driveTab.add("Left Voltage", 0)
      .withWidget(BuiltInWidgets.kGraph)
      .withPosition(3, 0)
      .getEntry();
  private NetworkTableEntry m_voltageU = 
    driveTab.add("Voltage U", 0)
      .withWidget(BuiltInWidgets.kGraph)
      .withPosition(6, 0)
      .getEntry();
  private NetworkTableEntry m_feedForward = 
    driveTab.add("FeedForward", 0)
      .withWidget(BuiltInWidgets.kGraph)
      .withPosition(6, 3)
      .getEntry();
  private NetworkTableEntry m_kalmanGain = 
    driveTab.add("Kalman Gain", 0)
      .withPosition(0, 3)
      .getEntry();

  /**
   * Constructs a StateSpaceDrive command
   *
   * @param speed Speed at which the robot should travel.
   * @param drive The Drivetrain
   */
  public StateSpaceDrive(double speed, Drivetrain drive) {
    m_drive = drive;
    m_speed = speed;
    addRequirements(drive);
    // The observer fuses our encoder data and voltage inputs to reject noise.
    m_observer = 
      new KalmanFilter<>(
          Nat.N2(),
          Nat.N2(),
          m_drive.getPlant(),
          VecBuilder.fill(3.0, 3.0), // How accurate we think our model is
          VecBuilder.fill(0.05, 0.05), // How accurate we think our encoder
          // data is
          0.020);

      // A LQR uses feedback to create voltage commands.
      m_controller =
        new LinearQuadraticRegulator<>(
            m_drive.getPlant(),
            // qelms. Velocity error tolerances
            VecBuilder.fill(Constants.ControlConstants.kMaxVelocityError, 
                            Constants.ControlConstants.kMaxControlEffort), 
            // relms. Control effort (voltage) tolerance
            VecBuilder.fill(Constants.ControlConstants.kMaxControlEffort, 
                            Constants.ControlConstants.kMaxControlEffort),
            0.020);

      // The state-space loop combines a controller, observer, feedforward and plant for easy control.
      m_loop = new LinearSystemLoop<>
        (m_drive.getPlant(), m_controller, m_observer, Constants.DriveConstants.maxVolts, 0.020);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.arcadeDrive(0, 0);
    m_drive.resetEncoders();
    m_loop.reset(Matrix.mat(Nat.N2(), Nat.N1()).fill(0,0));

    // Reset our last reference to the current state.
    m_lastProfiledReference =
        new TrapezoidProfile.State(m_drive.getLeftEncoderRate(), 
                                   m_drive.getLeftEncoderRate());

    // Report starting voltage to Shuffleboard
    m_leftVoltage.setDouble(m_loop.getU(0));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Sets the target speed of our drivetrain. 
    // This is similar to setting the setpoint of a PID controller.
    TrapezoidProfile.State goal = new TrapezoidProfile.State(m_speed, m_speed);
    // m_drive.m_loop.setNextR(VecBuilder.fill(m_speed, m_speed));

    m_lastProfiledReference =
        (new TrapezoidProfile(m_constraints, goal, m_lastProfiledReference)).calculate(0.020);

    m_loop.setNextR(m_lastProfiledReference.velocity, m_lastProfiledReference.velocity);

    // Correct our Kalman filter's state vector estimate with encoder data.
    // Get the current rate of the encoder. Units are distance per second as 
    // scaled by the value from setDistancePerPulse().
    double leftEncoderRate = m_drive.getLeftEncoderRate();
    double rightEncoderRate = m_drive.getRightEncoderRate();
    m_loop.correct(VecBuilder.fill(leftEncoderRate,rightEncoderRate));

    // Update our LQR to generate new voltage commands and use the voltages to 
    // predict the next state with our Kalman filter.
    m_loop.predict(0.020);

    // Send the new calculated voltage to the motors.
    // voltage = duty cycle * battery voltage, so
    // duty cycle = voltage / battery voltage
    double nextLeftVoltage = m_loop.getU(0);
    double nextRightVoltage = m_loop.getU(1);
    m_drive.setLeftVoltage(nextLeftVoltage);
    m_drive.setRightVoltage(-nextRightVoltage * DriveConstants.rightVoltsGain);

    // Put feedforward on Shuffleboard
    double ff = m_loop.getFeedforward().getUff(0);
    m_feedForward.setDouble(ff);

    // Put the calculated U voltage on Shuffleboard
    double leftVoltageU = m_loop.getController().getU(0);
    m_voltageU.setDouble(leftVoltageU);

    // Put the combined U voltage and feedforward voltage on Shuffleboard
    m_leftVoltage.setDouble(nextLeftVoltage);

    // Put Kalman Gain on Shuffleboard
    double kalmanGain = m_loop.getController().getK().get(0, 0);
    m_kalmanGain.setDouble(kalmanGain);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Put the setpoint to zero
    m_loop.setNextR(VecBuilder.fill(0.0, 0.0));

    // Report last voltage
    double nextLeftVoltage = m_loop.getU(0);
    m_leftVoltage.setDouble(nextLeftVoltage);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
