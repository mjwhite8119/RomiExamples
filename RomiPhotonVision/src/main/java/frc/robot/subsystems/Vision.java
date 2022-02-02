// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.sound.sampled.Line;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  // Constants such as camera and target height stored. Change per robot and goal!
  // final double CAMERA_HEIGHT_METERS = 0.092;
  final double CAMERA_HEIGHT_METERS = 0.10;
  final double TARGET_HEIGHT_METERS = 0.02;

  // Angle between horizontal and the camera.
  // final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(-18);
  final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(-26.5);

  // How far from the target we want to be
  // final double GOAL_RANGE_METERS = Units.feetToMeters(0.25);

  // Change this to match the name of your camera
  PhotonCamera m_camera = new PhotonCamera("mmal_service_16.1");
  private PhotonPipelineResult m_result;
  
  // Moving average filter used to smooth out target range
  private MedianFilter m_filter = new MedianFilter(10);
  private double m_range;
  private double m_lastRange = 0.0;

  private double m_lastPitch = 0.0;

  // Moving average filter used to check for a consistent target
  private MedianFilter m_targetFilter = new MedianFilter(10);
  private LinearFilter m_pitchFilter = LinearFilter.singlePoleIIR(1.0, 0.02);
  private int m_gotTarget = 0;
  /** 
   * Contructor
   * Creates a new RomiCamera that gets its data from PhotonVision
   * */
  public Vision() {
    m_result = m_camera.getLatestResult();
  }

  public double getYaw() {
    if (hasTargets()) {
      // double yawOffset = 0.12;
      // SmartDashboard.putNumber("Yaw with Offset", m_result.getBestTarget().getYaw() - yawOffset);
      return m_result.getBestTarget().getYaw();
    } else {
      System.out.println("NO Target!!!" );
      return 0.0;
    } 
  }

  public double getPitch() {
    if (hasTargets()) {
      m_lastPitch = m_result.getBestTarget().getPitch();
    }
    return m_pitchFilter.calculate(m_lastPitch);
  }

  public double getPitchRadians() {
    return Units.degreesToRadians(getPitch());
  }

  public double getRange() {
    if (hasTargets()) {
      m_range = PhotonUtils.calculateDistanceToTargetMeters(
                CAMERA_HEIGHT_METERS,
                TARGET_HEIGHT_METERS,
                CAMERA_PITCH_RADIANS,
                getPitchRadians());

      m_lastRange = m_range;          
      
    } 
    // Always return the last range and remove outliers
    double filteredRange = m_filter.calculate(m_lastRange);
    if (filteredRange < 0.5) {
      filteredRange = filteredRange * 1.2;
    }
    return filteredRange;
    // return m_filter.calculate(m_lastRange);
  }

  public double getSkew() {
    return m_result.getBestTarget().getSkew();
  }

  public boolean hasTargets() {
    m_result = m_camera.getLatestResult();
    return m_result.hasTargets();
  }

  public boolean lostTarget() {
    if (hasTargets()) {
      m_gotTarget = 1;
    } else {
      m_gotTarget = 0;
    }
    // If we got a target in the last 10 cycles return true
    double result = m_targetFilter.calculate(m_gotTarget);
    if (result > 0) {
      return false;
    } else {
      return true;
    }
  }

  public double getTargetHeight() {
    return TARGET_HEIGHT_METERS - CAMERA_HEIGHT_METERS;
  }

  public double getCameraPitch() {
    return CAMERA_PITCH_RADIANS;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // m_result = m_camera.getLatestResult();
  }
}
