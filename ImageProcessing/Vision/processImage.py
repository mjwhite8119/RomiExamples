import cv2
import numpy as np
from gripLineFollow import GripPipeline

def get_largest_contour(contours):
    return max(contours, key=lambda contour: cv2.boundingRect(contour)[2])


def start_process(cv_sink, nt_instance, output, imageWidth, imageHeight):
    targetData = nt_instance.getTable("targetData")

    pipeline = GripPipeline()

    while True:
        time, frame = cv_sink.grabFrame(np.zeros((imageHeight, imageWidth, 3), dtype=np.uint8))

        # Process the frame
        pipeline.process(frame)

        contours = pipeline.filter_contours_output
        if len(contours) > 0:
            contour = get_largest_contour(contours)
            moments = cv2.moments(contour)
            if moments["m00"] != 0:
                center_x = int(moments["m10"] / moments["m00"])
                x, y, width, height = cv2.boundingRect(contour)
                targetData.putNumber("centerX", center_x)
                targetData.putNumber("rectWidth", width)
                targetData.putNumber("rectHeight", height)

                # Draw a line on the frame
                cv2.line(frame, (center_x,0), (center_x, imageHeight), (0,0,255), 2)

        # Send processed images to Shuffleboard
        if time == 0: # There is an error
            output.notifyError(cv_sink.getError()) 
            continue
  
        output.putFrame(frame)
