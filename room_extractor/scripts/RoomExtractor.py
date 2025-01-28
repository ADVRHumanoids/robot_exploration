#!/usr/bin/env python

import cv2 as cv
import numpy as np
from skimage.feature import peak_local_max
from skimage.morphology import watershed
from scipy import ndimage

#ROS
import rospy
from nav_msgs.msg import OccupancyGrid
from std_srvs.srv import Trigger


def handle_extract_rooms(req):
   
    occupancy_msg = rospy.wait_for_message('/projected_map', OccupancyGrid, timeout=5)

    width = occupancy_msg.info.width
    height = occupancy_msg.info.height
    data = occupancy_msg.data  # This is a 1D array

    # Convert to a numpy array and reshape into a 2D grid
    grid_array = np.array(data, dtype=np.int8).reshape((height, width))

    # Normalize values for OpenCV
    cv_mat = np.zeros_like(grid_array, dtype=np.uint8)
    cv_mat[grid_array == -1] = 0  # Unknown
    cv_mat[grid_array == 0] = 255  # Free
    cv_mat[grid_array == 100] = 0  # Occupied

    cv_mat = cv.flip(cv_mat, 0)

    occupancy_map = cv.cvtColor(cv_mat,cv.COLOR_GRAY2BGR)
    gray = cv_mat

    # Acquire Occupancy
#    occupancy_map = data #cv.imread('map2.png')
    # gray = cv.cvtColor(occupancy_map,cv.COLOR_BGR2GRAY)
    _, binary_map = cv.threshold(gray,220,250,cv.THRESH_BINARY)

    kernel = np.ones((1,1),np.uint8)
    inflated_map = cv.erode(binary_map,kernel,iterations=1)

    distance_map = ndimage.distance_transform_edt(inflated_map)
    local_max = peak_local_max(distance_map, indices=False, min_distance=18, labels=inflated_map)

    # Perform connected component analysis then apply Watershed
    strcutre_label = [[1,1,1],
                      [1,1,1],
                      [1,1,1]]

    markers, _ = ndimage.label(local_max, structure=strcutre_label)
    labels = watershed(-distance_map, markers, mask=inflated_map)

    # Iterate through unique labels
    total_area = 0
    for label in np.unique(labels):
        if label == 0:
            continue

        # Create a mask
        mask = np.zeros(gray.shape, dtype="uint8")
        mask[labels == label] = 255

        # Find contours and determine contour area
        cnts = cv.findContours(mask.copy(), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0] if len(cnts) == 2 else cnts[1]
        c = max(cnts, key=cv.contourArea)
        print("ID: ", label,"Center: ", np.sum(c, axis=0)/len(c))

        area = cv.contourArea(c)
        total_area += area
        cv.drawContours(occupancy_map, [c], -1, (int(255*label/len(np.unique(labels))),int(255-255*label/len(np.unique(labels))),0), 1)
        
    print(total_area)
    cv.imshow('image', occupancy_map)
    cv.waitKey(0)
    cv.destroyAllWindows()

def main():

    rospy.init_node('RoomExtractor', anonymous=True)

    service = rospy.Service('extract_rooms', Trigger, handle_extract_rooms)

    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
