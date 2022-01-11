// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {

  // The Romi has the left and right motors set to
  // PWM channels 0 and 1 respectively
  private final RomiServo m_gripper = new RomiServo(ArmConstants.GRIPPER_PORT);
  private final RomiServo m_tilt = new RomiServo(ArmConstants.TILT_PORT);
  private final RomiServo m_lift = new RomiServo(ArmConstants.LIFT_PORT);
  // private final AnalogInput m_gripperRead = new AnalogInput(Constants.Arm.GRIPPER_FEEDBACK_PORT);

  public enum ArmPart {
    GRIPPER,
    TILT,
    LIFT
  }

  /** 
   * Constructor
   * Creates a new Arm subsystem
   * 
  */ 
  public Arm() {
    // Set the min/max range for each component
    m_gripper.setAngleRange(ArmConstants.GRIPPER_MIN, ArmConstants.GRIPPER_MAX);
    m_tilt.setAngleRange(ArmConstants.TILT_MIN, ArmConstants.TILT_MAX);
    m_lift.setAngleRange(ArmConstants.LIFT_MIN, ArmConstants.LIFT_MAX);

    // Set the default angle position
    m_tilt.setDefaultAngle(ArmConstants.TILT_MIN);
    m_lift.setDefaultAngle(ArmConstants.LIFT_MIN);
  }

  /** 
   * Increment tilt motor position
   * 
   * @param delta Amount to change motor position per loop cycle
   */
  public void tilt(double delta) {
    m_tilt.incrementServo(delta);
  }

  /**
   * Increment lift motor position
   * 
   * @param delta Amount to change motor position
   */
  public void lift(double delta) {
    m_lift.incrementServo(delta);
  }

  /** 
   * Increment gripper motor position
   * 
   * @param delta Amount to change motor position
   */ 
  public void gripper(double delta) {
    m_gripper.incrementServo(delta);
  }

  // Get lift motor position 
  public double getLiftPos() {
    return m_lift.getCurrentAngle();
  }

  // Get tilt motor position 
  public double getTiltPos() {
    return m_tilt.getCurrentAngle();
  }

  // Get lift motor position 
  public double getGripperPos() {
    return m_gripper.getCurrentAngle();
  }

  // // Get gripper motor position from feedback signal
  // public int getGripperFeedbackPos() {
  //   return m_gripperRead.getAverageValue();
  // }

  public boolean liftAtMax() {
    return m_lift.atMaxAngle();
  }

  public boolean liftAtMin() {
    return m_lift.atMinAngle();
  }

  public boolean tiltAtMax() {
    return m_tilt.atMaxAngle();
  }

  public boolean tiltAtMin() {
    return m_tilt.atMinAngle();
  }

  public boolean armAtMax() {
    return tiltAtMax() && liftAtMax();
  }

  public boolean armAtMin() {
    return tiltAtMin() && liftAtMin();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Gripper Position", getGripperPos());
    SmartDashboard.putNumber("Lift Position", getLiftPos());
    SmartDashboard.putNumber("Tilt Position", getTiltPos());

    SmartDashboard.putBoolean("Max Tilt", tiltAtMax());
    SmartDashboard.putBoolean("Min Tilt", tiltAtMin());
    SmartDashboard.putBoolean("Max Lift", liftAtMax());
    SmartDashboard.putBoolean("Min Tilt", liftAtMin());
  }
}
