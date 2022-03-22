// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.numbers.N2;

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
    public static final class DriveConstants {
        // Physical constants
        public static final double kCountsPerRevolution = 1440.0;
        public static final double kWheelDiameterMeter = 0.07;
        public static final double kMetersPerDegree = Math.PI * 0.141 / 360;
        public static final double kTrackwidthMeters = 0.142072613;
        public static final DifferentialDriveKinematics kDriveKinematics =
            new DifferentialDriveKinematics(kTrackwidthMeters);

        // Calibration for the right wheel voltage because it's much slower
        // than the left wheel on this robot.
        public static final double rightVoltsGain = 1.094;

        // Dynamical constants
        public static final double kMaxSpeedMetersPerSecond = 0.5;
        public static final double kMaxAccelMetersPerSecondSquared = 0.5;

        // The linear velocity gain, volts per (meter per second)
        public static final double kvVoltSecondsPerMeter = 9.7;
        // The angular velocity gain, volts per (radians per second)
        public static final double kvVoltSecondsPerRadian = 0.345;

        // The linear acceleration gain, volts per (meter per second squared).
        public static final double kaVoltSecondsSquaredPerMeter = 0.0737;
        // The angular acceleration gain, volts per (radians per second squared)
        public static final double kaVoltSecondsSquaredPerRadian = 0.0002;
        // public static final double kaVoltSecondsSquaredPerRadian = 0.00235;

        // Max volts that can be sent to the motors
        public static final double maxVolts = 7.0;

        // For distances PID
        public static final double kPDriveVel = 3.2;
        public static final double kIDriveVel = 0;
        public static final double kDDriveVel = 0;
        // public static final double kPDriveVel = 0.085;
        // public static final double kPDriveVel = 0.141;

        // Identify a standard differential drive drivetrain, given the drivetrain's kV and kA in both
        // linear (volts/(meter/sec) and volts/(meter/sec^2)) and angular (volts/(radian/sec) and
        // volts/(radian/sec^2)) cases. 
        // The Kv and Ka constants are found using the FRC Characterization toolsuite.
        // 
        // The plant holds a state-space model of our drivetrain. This system has the following properties:
        // 
        // State is: [left velocity, right velocity]
        // Inputs are [left voltage, right voltage]
        // Outputs are [left velocity, right velocity].       
        public static final LinearSystem<N2, N2, N2> kDrivetrainPlant =
            LinearSystemId.identifyDrivetrainSystem(kvVoltSecondsPerMeter, 
                                                    kaVoltSecondsSquaredPerMeter, 
                                                    kvVoltSecondsPerRadian, 
                                                    kaVoltSecondsSquaredPerRadian);       
    }

    public static final class ControlConstants {
        // qelms. Velocity error tolerances, in meters per second. Decrease this to more
        // heavily penalize state excursion, or make the controller behave more aggressively.
        public static final double kMaxVelocityError = 0.5;  
        // relms. Control effort (voltage) tolerance. Decrease this to more
        // heavily penalize control effort, or make the controller less aggressive. 6 is a good
        // starting point because that is the (approximate) maximum voltage of a battery.
        public static final double kMaxControlEffort = 6.0;
    }

    public final class Joystick {
        // Button mapping for a PS3 gamepad. 
        public static final int SELECT = 1;
        public static final int A = 2;
        public static final int B = 3;
        public static final int START = 4;
        public static final int TOP_DIR = 5;
        public static final int RIGHT_DIR = 6;
        public static final int BOTTOM_DIR = 7;
        public static final int LEFT_DIR = 8;
        public static final int L2_BUTTON = 9;
        public static final int R2_BUTTON = 10;
        public static final int LEFT_ANALOG = 11;
        public static final int RIGHT_ANALOG = 12;
        public static final int TRIANGLE_BUTTON = 13;
        public static final int CIRCLE_BUTTON = 14;
        public static final int CROSS_BUTTON = 15;
        public static final int SQUARE_BUTTON = 16;
        public static final int UNKNOWN_BUTTON = 17;
    }
}
