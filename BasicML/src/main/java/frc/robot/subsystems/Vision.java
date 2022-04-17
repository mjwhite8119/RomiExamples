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

import frc.robot.detections.Detections;

import java.util.Collections;
import java.util.Map;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;

public class Vision extends SubsystemBase {
    private NetworkTable m_tableML = NetworkTableInstance.getDefault().getTable("ML");
    private NetworkTableEntry deviceEntry;
    private NetworkTableEntry resolutionEntry;
    private NetworkTableEntry fspEntry;
    private NetworkTableEntry detectionsEntry;

    Detections m_detections = new Detections();
    Map<String, Integer> emptyMap = Collections.emptyMap();

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
        parseDetections(detectionsEntry.getString("No detections"));
        
        fspEntry = m_tableML.getEntry("fsp");
        SmartDashboard.putNumber("FSP", fspEntry.getDouble(0.0));

        SmartDashboard.putNumber("ymin", getYMin());
        // SmartDashboard.putNumber("xmin", getXMin());
        // SmartDashboard.putNumber("ymax", getYMax());
        // SmartDashboard.putNumber("xmax", getXMax());
    }

    public void parseDetections(String json) {
        //create ObjectMapper instance
		ObjectMapper mapper = new ObjectMapper();
        //JSON string to Java Object
        try {
            m_detections = mapper.readValue(json, Detections.class);
        } catch (JsonProcessingException e) {
            System.out.println(e);
            m_detections.box = emptyMap;
        } 	    
    }

    public String getLabel() {
        return m_detections.label;
    }

    public int getConfidence() {
        return m_detections.confidence;
    }

    public Map<String, Integer> getBox() {
        // System.out.println(m_detections.box);
        return m_detections.box; 
    }

    public Integer getYMin() {
        Integer ymin = getBox().get("ymin");
        if (ymin != null) {
            return ymin;
        }
        return -1;
    }

    public Integer getXMin() {
        return getBox().get("xmin");
    }
    
    public Integer getYMax() {
        return getBox().get("ymax");
    }

    public Integer getXMax() {
        return getBox().get("xmax");
    }

}

