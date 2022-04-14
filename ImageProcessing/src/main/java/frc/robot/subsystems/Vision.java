/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    NetworkTableEntry visionEntry;
    NetworkTableEntry centerEntry;
    NetworkTableEntry rectWidthEntry;
    NetworkTableEntry rectHeightEntry;
    private int centerX = -1;
    private int width = -1;

    private int rectWidth;
    private int rectHeight;

    public Vision() {
        centerEntry = NetworkTableInstance.getDefault().getTable("targetData").getEntry("centerX");
        rectWidthEntry = NetworkTableInstance.getDefault().getTable("targetData").getEntry("rectWidth");
        rectHeightEntry = NetworkTableInstance.getDefault().getTable("targetData").getEntry("rectHeight");
    }

    @Override
    public void periodic() {
        // Data from Python 
        centerX = (int)centerEntry.getDouble(0.0);
        rectWidth = (int)rectWidthEntry.getDouble(0.0);
        rectHeight = (int)rectHeightEntry.getDouble(0.0);

        SmartDashboard.putNumber("X Center", centerX);
        SmartDashboard.putNumber("Rect Width", rectWidth);
        SmartDashboard.putNumber("Rect Height", rectHeight);
        SmartDashboard.putNumber("Rect Area", rectHeight * rectWidth);
    }

    public int getWidth() {
        return width;
    }

    public int getCenterX() {
        return centerX == -1 ? 320: centerX;
    }

    public int getRectWidth() {
        return rectWidth;
    }

    public int getRectHeight() {
        return rectHeight;
    }

    public int getRectArea() {
        return rectWidth * rectHeight;
    }
}

