import cv2
import numpy as np

def start_process(cv_sink, nt_instance, output, imageWidth, imageHeight):
    
    targetData = nt_instance.getTable("targetData")

    center_x = int(imageWidth / 2)
    center_y = int(imageHeight / 2)

    while True:
        time, frame = cv_sink.grabFrame(np.zeros((imageHeight, imageWidth, 3), dtype=np.uint8))

        targetData.putNumber("centerX", center_x)
        targetData.putNumber("centerY", center_y)

        # Draw a circle on the frame
        cv2.circle(frame, (center_x,center_y), 5, (0,0,255), 2) 

        # Send processed images to Shuffleboard
        if time == 0: # There is an error
            output.notifyError(cv_sink.getError()) 
            continue
  
        output.putFrame(frame)
