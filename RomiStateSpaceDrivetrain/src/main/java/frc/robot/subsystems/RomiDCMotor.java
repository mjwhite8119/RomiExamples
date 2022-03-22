// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;

/** Holds the constants for a DC motor. */
public class RomiDCMotor {
    @SuppressWarnings("MemberName")
    public final double nominalVoltageVolts;

    @SuppressWarnings("MemberName")
    public final double stallTorqueNewtonMeters;

    @SuppressWarnings("MemberName")
    public final double stallCurrentAmps;

    @SuppressWarnings("MemberName")
    public final double freeCurrentAmps;

    @SuppressWarnings("MemberName")
    public final double freeSpeedRadPerSec;

    @SuppressWarnings("MemberName")
    public final double rOhms;

    @SuppressWarnings("MemberName")
    public final double KvRadPerSecPerVolt;

    @SuppressWarnings("MemberName")
    public final double KtNMPerAmp;

    /**
     * Constructs a DC motor.
     *
     * @param nominalVoltageVolts     Voltage at which the motor constants were
     *                                measured.
     * @param stallTorqueNewtonMeters Current draw when stalled.
     * @param stallCurrentAmps        Current draw when stalled.
     * @param freeCurrentAmps         Current draw under no load.
     * @param freeSpeedRadPerSec      Angular velocity under no load.
     * @param numMotors               Number of motors in a gearbox.
     */
    public RomiDCMotor(double nominalVoltageVolts, double stallTorqueNewtonMeters, double stallCurrentAmps,
            double freeCurrentAmps, double freeSpeedRadPerSec, int numMotors) {
        this.nominalVoltageVolts = nominalVoltageVolts;
        this.stallTorqueNewtonMeters = stallTorqueNewtonMeters * numMotors;
        this.stallCurrentAmps = stallCurrentAmps * numMotors;
        this.freeCurrentAmps = freeCurrentAmps * numMotors;
        this.freeSpeedRadPerSec = freeSpeedRadPerSec;

        this.rOhms = nominalVoltageVolts / this.stallCurrentAmps;
        this.KvRadPerSecPerVolt = freeSpeedRadPerSec / (nominalVoltageVolts - rOhms * this.freeCurrentAmps);
        this.KtNMPerAmp = this.stallTorqueNewtonMeters / this.stallCurrentAmps;
    }

    /**
     * Estimate the current being drawn by this motor.
     *
     * @param speedRadiansPerSec The speed of the rotor.
     * @param voltageInputVolts  The input voltage.
     */
    public double getCurrent(double speedRadiansPerSec, double voltageInputVolts) {
        return -1.0 / KvRadPerSecPerVolt / rOhms * speedRadiansPerSec + 1.0 / rOhms * voltageInputVolts;
    }

    /**
     * Return a gearbox of Romi motors.
     *
     * @param numMotors Number of motors in the gearbox.
     */
    public static RomiDCMotor getRomi(int numMotors) {
        return new RomiDCMotor(12, 2.42, 133, 2.7, Units.rotationsPerMinuteToRadiansPerSecond(5310), numMotors);
    }

}
