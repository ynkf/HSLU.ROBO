#!/usr/bin/python3
import os
from pathlib import Path
import rospy
import cv2 as cv
import time
import numpy as np
import math
from enum import Enum
from wheel_command_publisher import WheelCommandPublisher
from camera_subscriber import CameraSubscriber
from wheel_encoder import WheelEncoderTracker
from pathfinder import Pathfinder, Dirs

BASE_SPEED = 0.3

VERBOSE_DEBUG = True

class Labyrinth:
    def __init__(self, robot_name, path):
        self.wcp = WheelCommandPublisher(robot_name)
        self.camera = CameraSubscriber(robot_name)
        self.encoder = WheelEncoderTracker(robot_name)
        self.path = path
        self.path_index = 0
        
    def debug_state(self, images = []):
        now = time.time_ns()
        rospy.logdebug(f"Debuging state at time {now}:")
        rospy.logdebug(f"\tCurrent path index: {self.path_index}")
        Path("./debug").mkdir(parents=True, exist_ok=True)
        for i, image in enumerate(images):
            cv.imwrite(f"./debug/{now}_{i}.png", image)
        
        
    def do_update(self):
        images = []
        
        img = self.camera.get_image_array()
        if img is None:
            rospy.loginfo("No image found. Skipping...")
            return True
        images.append(img)
        
        # IMAGE PROCESSING SECTION
        # ========================
        gray = cv.cvtColor(img, cv.COLOR_RGB2GRAY)
        
        SCALE = (100, 75)
        scaled = cv.resize(gray, SCALE, interpolation=cv.INTER_NEAREST)
        height, width = scaled.shape

        CROP_AMOUNT = 0.6
        cropped = scaled[int(height * CROP_AMOUNT):height, :]
        height, width = cropped.shape

        thresh = cv.adaptiveThreshold(cropped, 255, cv.ADAPTIVE_THRESH_GAUSSIAN_C, cv.THRESH_BINARY, 15, 1)
        images.append(thresh)
        
        # FINDING PATH SECTION
        # ====================
        contours, _ = cv.findContours(thresh, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        
        if len(contours) == 0:
            rospy.logwarn("Found no contours in current frame. Stopping...")
            self.debug_state(images)
            return False
        
        debug_contours = None
        max_score = -1
        path = None
        # Find the brightest contour which is our white line
        for contour in contours:
            # Create mask
            mask = np.zeros_like(cropped, dtype=np.uint8)
            mask = cv.drawContours(mask, [contour], -1, (255), thickness=cv.FILLED)
            # Calculate the mean brightness using the mask
            mean_val = cv.mean(cropped, mask=mask)[0]  # Only take the first value since the image is grayscale
            
            m = cv.moments(contour)
            area = m['m00']
            if area == 0:
                continue
            score = math.log10(area) * math.pow(mean_val / 255, 2)
            
            if score > max_score:
                max_score = score
                path = contour
                
            if VERBOSE_DEBUG:
                if debug_contours is None:
                    debug_contours = cv.cvtColor(cropped.copy(), cv.COLOR_GRAY2BGR)
                    images.append(debug_contours)
                x = int(m['m10'] / m['m00'])  # X coordinate of the paths center
                y = int(m['m01'] / m['m00'])  # Y coordinate of the paths center
                debug_contours = cv.line(debug_contours, (x-1, y-1), (x+1, y+1), (score*500, 0, 0))
                debug_contours = cv.line(debug_contours, (x+1, y-1), (x-1, y+1), (score*500, 0, 0))
                
        if path is None:        
            rospy.logwarn("Found no contour. Stopping...")
            self.debug_state(images)
            return False
        
        if VERBOSE_DEBUG:
            debug_cont = cv.cvtColor(cropped.copy(), cv.COLOR_GRAY2BGR)
            debug_cont = cv.drawContours(debug_cont, [path], -1, (255, 0, 0), thickness=cv.FILLED)
            debug_cont = cv.line(debug_cont, (x-1, y-1), (x+1, y+1), 0)
            debug_cont = cv.line(debug_cont, (x+1, y-1), (x-1, y+1), 0)
            images.append(debug_cont)
            
        M = cv.moments(path)
        path_x = int(M['m10'] / M['m00'])  # X coordinate of the paths center
        
        def normalize_pixel(x):
            return (x / (width - 1)) * 2 - 1
        
        box_left, box_top, box_width, box_height = cv.boundingRect(path)
        box_right = box_left + box_width
        box_center_x = box_left / (box_width - 1)
        
        if VERBOSE_DEBUG:
            box_img = cv.cvtColor(cropped.copy(), cv.COLOR_GRAY2BGR)
            box_img = cv.rectangle(box_img, (box_left, box_top), (box_left + box_width, box_top + box_height), (255, 0, 0))
            images.append(box_img)

        
        # NAVIGATION SECTION
        # ====================
        possible_dirs = set()

        if box_width > width * 0.5:
            MIN_OFFSET = 0.125
            if box_width > width * 0.8 or box_center_x < 0.5 - MIN_OFFSET:
                possible_dirs.add(Dirs.LEFT)
            if box_width > width * 0.8 or box_center_x > 0.5 + MIN_OFFSET:
                possible_dirs.add(Dirs.RIGHT)
        
        # If there is a path forwards and there's a crossing, forwards is an action
        if box_height >= height and len(possible_dirs.intersection([Dirs.LEFT, Dirs.RIGHT])) > 0:
            possible_dirs.add(Dirs.FORWARDS)
        # We have reached the end of the path
        elif box_height < height and len(possible_dirs) == 0:
            rospy.loginfo("Reached the end of the path.")
            self.debug_state(images)
            return False
        
        
        next_dir = self.path[self.path_index] if self.path_index < len(self.path) else None
        # No action, simply drive along the path
        if next_dir not in possible_dirs:
            slowdown = normalize_pixel(path_x) * BASE_SPEED
            left = min(BASE_SPEED + slowdown, BASE_SPEED)
            right = min(BASE_SPEED - slowdown, BASE_SPEED)
            self.wcp.set_left_right(left, right)
        # We have a possible action
        else:
            rospy.loginfo(f"Reached crossing, going {next_dir}")
            self.debug_state(images)
            if next_dir == Dirs.LEFT:
                self.move_and_turn(-0.25)
            elif next_dir  == Dirs.RIGHT:
                self.move_and_turn(0.25)
            elif next_dir == Dirs.FORWARDS:
                self.move_and_turn(0)
            self.debug_state(images)
            
            self.path_index += 1

        self.debug_state()
        return True
    
    def move_and_turn(self, rot):
        # Move to center
        self.wcp.set_left_right(0.2, 0.2)
        rospy.sleep(0.8)
        # Turn
        if rot == 0:
            return
        
        def current_ticks():
            if rot < 0:
                return self.encoder.right_ticks
            return self.encoder.left_ticks
        
        tick_diff = 360 * abs(rot)
        init_ticks = current_ticks()
        
        if rot < 0:
            self.wcp.set_left_right(0, 0.2)
        else:
            self.wcp.set_left_right(0.2, 0)
            
        while current_ticks() - tick_diff < init_ticks:
            rospy.sleep(1/30)
        self.wcp.set_left_right(0, 0)

    
    def run(self):
        rospy.loginfo(f"Starting Labyrinth loop")
        try:
            while not rospy.is_shutdown():
                stop = not self.do_update()
                if stop:
                    break
        finally:
            self.wcp.set_left_right(0, 0)
        rospy.loginfo(f"Labyrinth loop has ended.")


if __name__ == "__main__":
    robot_name = os.environ["VEHICLE_NAME"]
    rospy.loginfo(f"Starting up Labyrinth Script for robot: {robot_name}")
    rospy.init_node("labyrinth_script", anonymous=False)

    pf = Pathfinder()
    start_node = 'DeadEnd2'
    end_node = 'DeadEnd12'
    path_edges = pf.find_shortest_path(start_node, end_node)

    if path_edges:
        # Print the sequence of nodes
        nodes_in_path = [edge.origin for edge in path_edges] + [path_edges[-1].target]
        print(f"Shortest path from {start_node} to {end_node}: {nodes_in_path}")

        # Get directions along the path
        directions = pf.get_directions(path_edges)
        print(f"Directions: {directions}")
    else:
        print(f"No path found from {start_node} to {end_node}.")

    

    lab = Labyrinth(robot_name, directions)
    
    def shutdown_callback():
        lab.wcp.set_left_right(0, 0)
    rospy.on_shutdown(shutdown_callback)
    
    rospy.loginfo(f"Labyrinth initialized")
    lab.run()
    