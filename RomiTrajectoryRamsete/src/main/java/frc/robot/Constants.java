// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DrivetrainConstants
   {

    // -------- Physical Constants -----------------
    public static final double kCountsPerRevolution = 1440.0;
    public static final double kWheelDiameterMeters = 0.07;
    public static final double kMetersPerDegree = Math.PI * 0.141 / 360;
    public static final double kTrackwidthMeters = 0.142072613;
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);

    // Calibration for the right wheel voltage because it's much slower
    // than the left wheel on this robot.
    // public static final double rightVoltsGain = 1.094; // Romi
    public static final double rightVoltsGain = 1.0; // Romi-2

    // -------- Dynamical constants --------------------

    // Max speed and acceleration of the robot
    public static final double kMaxSpeedMetersPerSecond = 0.5;
    public static final double kMaxAccelMetersPerSecondSquared = 0.5;

    // The linear inertia gain, volts
    public static final double ksVolts = 0.461;
    // The linear velocity gain, volts per (meter per second)
    // Increase this if you drive short
    public static final double kvVoltSecondsPerMeter = 6.93;
    // public static final double kvVoltSecondsPerMeter = 9.7;
    // The linear acceleration gain, volts per (meter per second squared).
    public static final double kaVoltSecondsSquaredPerMeter = 0.0737;

    // Setup constraints for feedforward and kinematics
    public static final SimpleMotorFeedforward kFeedForward = 
      new SimpleMotorFeedforward(ksVolts, 
                                kvVoltSecondsPerMeter, 
                                kaVoltSecondsSquaredPerMeter);

    public static final DifferentialDriveVoltageConstraint kAutoVoltageConstraint =
      new DifferentialDriveVoltageConstraint(
          kFeedForward,
          kDriveKinematics,
          10);

    // -------- Trajectory constants --------------------

    // Setup trajectory constraints
    public static final TrajectoryConfig kTrajectoryConfig =
      new TrajectoryConfig(kMaxSpeedMetersPerSecond, 
                           kMaxAccelMetersPerSecondSquared)
          .setKinematics(kDriveKinematics)
          .addConstraint(kAutoVoltageConstraint);

    // -------- PID constants --------------------
    public static final double kPDriveVel = 0.125;
    public static final double kIDriveVel = 0;
    public static final double kDDriveVel = 0;

    public static final double kPDriveVelLeft = 0.125;
    public static final double kPDriveVelRight = 0.125;
  }

  public static final class ControlConstants {
    

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
  }
}
