#!/usr/bin/python3

import cv2
import numpy as np
import matplotlib.pyplot as plt
import scipy
import math

R_RESOLUTION = 1 # in px
THETA_RESOLUTION = np.pi/180 # in rads
CROP = 1500 # how many px's to crop off the edges
N_HYPOTHESES = 4 # how many merging hypotheses to keep track of

import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import TransformStamped, PoseArray
import tf2_ros
import tf_conversions

def shiftColors(a):
    "Shift colors back to normal ones"
    if a == 255:
        return 205
    if a == 0:
        return 255
    if a == 100:
        return 0
    return a


def houghSpectrum(gray_img):
    # Apply edge detection method on the image
    walls = cv2.threshold(gray_img, 206, 255, cv2.THRESH_BINARY)[1]

    edges = cv2.Canny(walls, 50, 150, apertureSize=3)


    # This returns an array of r and theta values
    lines = cv2.HoughLines(edges, R_RESOLUTION, THETA_RESOLUTION, 50)

    counts = [0] * int(2 * np.pi / THETA_RESOLUTION)
    bin_edges = [x for x in np.arange(0, 2* np.pi, THETA_RESOLUTION)]

    assert(len(counts) == len(bin_edges))

    for r_theta in lines:
        r, theta = np.array(r_theta[0], dtype=np.float64)
        if r < 0:
            theta += np.pi
        i = int(theta / THETA_RESOLUTION)
        counts[i] += 1

    #plt.bar(bin_edges, counts, width=0.3)
    #plt.plot(bin_edges, counts)
    #plt.show()

    return counts

def circularCrossCorrelation(hs1, hs2):
    assert(len(hs1) == len(hs2))
    theta_s = int(2 * np.pi / THETA_RESOLUTION)
    ccc = [0] * theta_s
    for k in range(theta_s):
        for i in range(theta_s):
            ccc[k] += hs1[i] * hs2[(i + k) % theta_s]
    return ccc

def xyCrossCorrelation(spec1, spec2):
    assert(len(spec1) == len(spec2))
    corr = scipy.signal.correlate(spec1, spec2, mode='same')
    return np.argmax(corr) - (len(spec1) // 2)


def localMaxima(arr, n):
    arr = list(enumerate(arr))
    arr.sort(key=lambda x: x[1], reverse=True)
    return [x[0] * THETA_RESOLUTION for x in arr[:n]]

def xySpec(occ_map):
    # slow
    h, w, _ = occ_map.shape
    yspec = [0] * h
    xspec = [0] * w

    for x in range(w):
        for y in range(h):
            occupied = int(occ_map[y, x, 0] < 205)
            if occ_map[y, x, 0] not in [0, 205, 255]:
                print(occ_map[y,x,0])
            xspec[x] += occupied  
            yspec[y] += occupied  
    
    return xspec, yspec

def moveImg(img, angle, dx, dy):
    height, width = img.shape[:2]
    center = (width/2, height/2)
    rotate_matrix = cv2.getRotationMatrix2D(center=center, angle=angle*180/np.pi, scale=1)
    rotate_matrix = np.vstack([rotate_matrix, [0.0, 0.0, 1.0]])
    trans_matrix = np.array([[1.0, 0.0, dx], [0.0, 1.0, dy], [0.0, 0.0, 1.0]])
    matrix = trans_matrix @ rotate_matrix
    rotated_image = cv2.warpPerspective(src=img, M=matrix, dsize=(width, height), flags=cv2.INTER_NEAREST)
    return rotated_image, matrix

def acceptanceIndex(img1, img2):
    # slow
    h, w, _ = img1.shape
    agreement = 0.0
    disagreement = 0.0

    for x in range(w):
        for y in range(h):
            if img1[y, x, 0] == 205 or img2[y, x, 0] == 205:
                continue
            if img1[y, x, 0] == img2[y, x, 0]:
                agreement += 1
            else:
                disagreement += 1
    
    return 0 if agreement == 0 else agreement / (agreement + disagreement), agreement



class MapMergeNode(object):
    def __init__(self, robot_names, map_topic_name="map"):
        self.tf2_br = tf2_ros.TransformBroadcaster()
        self.tfBuffer = tf2_ros.Buffer()
        # self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.suggested_angle = 0

        self.map_subscribers = []
        self.detect_subscribers = []
        self.map_topic_name = map_topic_name
        self.maps = {}
        for robot in robot_names:
            self.map_subscribers.append(rospy.Subscriber(f"/{robot}/{map_topic_name}", OccupancyGrid, self._map_callback, robot))
            self.detect_subscribers.append(rospy.Subscriber(f"/{robot}/detected_robots", PoseArray, self._detect_callback, robot))
            self.maps[robot] = None


        rospy.sleep(2)
        self.merge_maps(None)
        #rospy.Timer(rospy.Duration(20), self.merge_maps)
    
    def _detect_callback(self, poses, robot_name):
        if '0' in robot_name:
            other_name = 'robot_1'
        else:
            other_name = 'robot_0'

        try:
            trans1 = self.tfBuffer.lookup_transform( f'{other_name}', f'{robot_name}/base_link', rospy.Time())
            trans2 = self.tfBuffer.lookup_transform(f'{other_name}/base_link', f'{robot_name}/base_link', rospy.Time())
            measured = (trans1.transform.translation.x, trans1.transform.translation.y)
            real = (-trans2.transform.translation.x, -trans2.transform.translation.y)
            self.suggested_angle = math.acos(np.dot(measured, real) / (np.linalg.norm(measured) * np.linalg.norm(real)))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            pass
        
        

    def _map_callback(self, map, robot_name):
        self.maps[robot_name] = map

    def merge_maps(self, _event):
        rospy.loginfo("Merging maps...")
        (robot1, map1), (robot2, map2) = list(self.maps.items())
        # convert int8 map array into uint8 matrix
        # this means 255 = unknown, 100 = occupied, 0 = free
        grid1 = np.reshape(map1.data, (map1.info.width, map1.info.height)).astype('uint8')[CROP:-CROP, CROP:-CROP]
        grid2 = np.reshape(map2.data, (map2.info.width, map2.info.height)).astype('uint8')[CROP:-CROP, CROP:-CROP]

        vfunc = np.vectorize(shiftColors, otypes=['uint8'])
        grid1 = vfunc(grid1)
        grid2 = vfunc(grid2)

        img1 = cv2.cvtColor(grid1, cv2.COLOR_GRAY2BGR)
        img2 = cv2.cvtColor(grid2, cv2.COLOR_GRAY2BGR)
        # cv2.imshow('image', img1)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()

        rospy.loginfo(img1.shape)
        rospy.loginfo(img2.shape)

        hs1 = houghSpectrum(img1)

        # align coords for better performance
        best_rotation = localMaxima(hs1, 1)[0]
        img1, _ = moveImg(img1, best_rotation, 0, 0)
        
        hs1 = houghSpectrum(img1)
        hs2 = houghSpectrum(img2)
    
        ccc = circularCrossCorrelation(hs1, hs2)
        maxima = localMaxima(ccc, N_HYPOTHESES)
        #maxima.append(self.suggested_angle)
        print(maxima)
        xspec1, yspec1 = xySpec(img1)

        best_w = 0
        best_angle = 0
        best_transform = None
        best_trans = (0, 0)
        best_image = None
        best_agreement = 0
        for base_angle in maxima:
            for angle in [base_angle - THETA_RESOLUTION/3, base_angle, base_angle + THETA_RESOLUTION/3]:
                img3, t = moveImg(img2, angle, 0, 0)
                xspec3, yspec3 = xySpec(img3)
                dx = xyCrossCorrelation(xspec1, xspec3)
                dy = xyCrossCorrelation(yspec1, yspec3)
                tm2, t = moveImg(img2, angle, dx, dy)
                w, agreement = acceptanceIndex(img1, tm2)
                if w > best_w:
                    best_w = w
                    best_angle = angle
                    best_trans = (dx, dy)
                    best_transform = t
                    best_image = tm2
                    best_agreement = agreement

                # dst = cv2.addWeighted(img1, 0.5, tm2, 0.5, 0.0)
                # cv2.imwrite(f"{w}.jpg", dst)
        
        dst = cv2.addWeighted(img1, 0.5, best_image, 0.5, 0.0)
        cv2.imwrite(f"merged_map_w_{best_w:.4f}.png", dst)
        # cv2.imshow('image', dst)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
        rospy.loginfo(f"Best w: {best_w}, with angle: {best_angle - best_rotation}. Suggested: {maxima}")


        # broadcast a transform from map1 to map2
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = f"{robot1}/{self.map_topic_name}"
        t.child_frame_id = f"{robot2}/{self.map_topic_name}"
        t.transform.translation.x = best_trans[0] 
        t.transform.translation.y = best_trans[1] 
        t.transform.translation.z = 0.0
        q = tf_conversions.transformations.quaternion_from_euler(0, 0, best_angle)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # broadcast transforms
        self.tf2_br.sendTransform(t)

def getHeading(q):
    """
    Get the robot heading in radians from a Quaternion representation.
    
    :Args:
        | q (geometry_msgs.msg.Quaternion): a orientation about the z-axis
    :Return:
        | (double): Equivalent orientation about the z-axis in radians
    """
    yaw = math.atan2(2 * (q.x * q.y + q.w * q.z),
                     q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z)
    return yaw


if __name__ == "__main__":
    rospy.init_node("map_merge")
    node = MapMergeNode(["robot_0", "robot_1"])
    rospy.spin()
