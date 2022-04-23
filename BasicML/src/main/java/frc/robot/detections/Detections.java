package frc.robot.detections;

import java.util.Collections;
import java.util.Map;

/**
 * This class creates a mapping onto the Yolo object detections
 * that take place on the Raspberry Pi with the OAK-D camera.
 * @param label The name of the object detected
 * @param box The bounding bow around the detected object
 * @param confidence The confidence level of the detected object.
 * 
 * {
 *  "label": labelMap[detection.label], 
 *  "box": {"ymin": ymin, "xmin": xmin, "ymax": ymax, "xmax": xmax}, 
 *  "spacial": {"X": x_coord, "Y": y_coord, "Z": z_coord},
 *  "confidence": confidence
 * }  
 */
public class Detections {
    public String label;
    public Map<String, Double> box = Collections.emptyMap();
    public Map<String, Integer> spacial = Collections.emptyMap();
    public int confidence;                         
}