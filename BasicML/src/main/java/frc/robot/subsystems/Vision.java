/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    private NetworkTable m_tableML = NetworkTableInstance.getDefault().getTable("ML");
    private NetworkTableEntry deviceEntry;
    private NetworkTableEntry resolutionEntry;
    private NetworkTableEntry fspEntry;
    private NetworkTableEntry detectionsEntry;
    private int centerX = -1;
    private int width = -1;

    private int rectWidth;
    private int rectHeight;

    public Vision() {
        
        deviceEntry = m_tableML.getEntry("device");
        resolutionEntry = m_tableML.getEntry("resolution");
        
        SmartDashboard.putString("OAK Device", deviceEntry.getString("Not getting device"));
        SmartDashboard.putString("Image Resolution:", resolutionEntry.getString("No resolution"));
    }

    @Override
    public void periodic() {
        // Data from Python 
        detectionsEntry = m_tableML.getEntry("detections");
        fspEntry = m_tableML.getEntry("fsp");
        SmartDashboard.putNumber("FSP", fspEntry.getDouble(0.0));
        // System.out.println(fspEntry.getDouble(0.0));

        // SmartDashboard.putNumber("X Center", centerX);
        // SmartDashboard.putNumber("Rect Width", rectWidth);
        // SmartDashboard.putNumber("Rect Height", rectHeight);
        // SmartDashboard.putNumber("Rect Area", rectHeight * rectWidth);
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

