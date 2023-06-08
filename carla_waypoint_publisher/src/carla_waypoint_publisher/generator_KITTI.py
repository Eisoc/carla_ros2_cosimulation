#!/usr/bin/env python3

# from ast import Pass
import glob
import os
import sys
import cv2
import time
import h5py
# from pathlib import Path
from PIL import Image
try:
    '''
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
    '''
    sys.path.append('/home/bing/carla-ros-bridge/src/ros-bridge/carla_waypoint_publisher/src/carla_waypoint_publisher/')
    sys.path.append("/home/bing/carla-ros-bridge/src/ros-bridge/install/carla_waypoint_publisher/lib/python3.8/site-packages/carla_waypoint_publisher/")

except IndexError:
    pass

import carla
from carla import VehicleLightState as vls

#from modules import ply
import logging
import queue
import struct
import math
import numpy as np
import random
import threading

batch = []#Batch stores all vehicles except MyCar

def sensor_callback(ts, sensor_data, sensor_queue):
    sensor_queue.put(sensor_data)
def location_callback(location_data, location_queue):
    location_queue.put(location_data)

class Sensor:
    initial_ts = 0.0
    initial_loc = carla.Location()
    initial_rot = carla.Rotation()

    def __init__(self, vehicle, world, actor_list, folder_output, transform):
        self.queue = queue.Queue()
        self.bp = self.set_attributes(world.get_blueprint_library())
        self.sensor = world.spawn_actor(self.bp, transform, attach_to=vehicle)
        actor_list.append(self.sensor)
        self.sensor.listen(lambda data: sensor_callback(data.timestamp - Sensor.initial_ts, data, self.queue))
        self.sensor_id = self.__class__.sensor_id_glob;
        self.__class__.sensor_id_glob += 1 
        self.folder_output = folder_output
        self.ts_tmp = 0

class Camera(Sensor):
    def __init__(self, vehicle, world, actor_list, folder_output, transform):
        Sensor.__init__(self, vehicle, world, actor_list, folder_output, transform)
        self.sensor_frame_id = 0
        self.frame_output = self.folder_output+"/data_%s" %str.lower(self.__class__.__name__)
        os.makedirs(self.frame_output) if not os.path.exists(self.frame_output) else [os.remove(f) for f in glob.glob(self.frame_output+"/*") if os.path.isfile(f)]

        with open(self.folder_output+"/full_ts_camera.txt", 'w') as file:
            file.write("# frame_id timestamp\n")

        print('created %s' % self.sensor)

    def save(self, color_converter=carla.ColorConverter.Raw):
        while not self.queue.empty():
            data = self.queue.get()

            ts = data.timestamp-Sensor.initial_ts
            if ts - self.ts_tmp > 0.11 or (ts - self.ts_tmp) < 0: #check for 10Hz camera acquisition
                print("[Error in timestamp] Camera: previous_ts %f -> ts %f" %(self.ts_tmp, ts),data.timestamp,Sensor.initial_ts)
                # sys.exit()
            self.ts_tmp = ts

            file_path = self.frame_output+"/%05d_%d.png" %(self.sensor_frame_id, self.sensor_id)
            x = threading.Thread(target=data.save_to_disk, args=(file_path, color_converter))
            x.start()
            print("Export : "+file_path)

            if self.sensor_id == 0:
                with open(self.folder_output+"/full_ts_camera.txt", 'a') as file:
                    file.write(str(self.sensor_frame_id)+" "+str(data.timestamp - Sensor.initial_ts)+"\n") 
            self.sensor_frame_id += 1

class RGB_left(Camera):
    sensor_id_glob = 0

    def __init__(self, vehicle, world, actor_list, folder_output, transform):
        Camera.__init__(self, vehicle, world, actor_list, folder_output, transform)

    def set_attributes(self, blueprint_library):
        camera_bp = blueprint_library.find('sensor.camera.rgb')

        #camera_bp.set_attribute('image_size_x', '1392')
        #camera_bp.set_attribute('image_size_y', '1024')
        camera_bp.set_attribute('image_size_x', '696')
        camera_bp.set_attribute('image_size_y', '512')
        camera_bp.set_attribute('fov', '72') #72 degrees # Always fov on width even if width is different than height
        camera_bp.set_attribute('enable_postprocess_effects', 'True')
        camera_bp.set_attribute('sensor_tick', '0.10') # 10Hz camera
        camera_bp.set_attribute('gamma', '2.2')
        camera_bp.set_attribute('motion_blur_intensity', '0')
        camera_bp.set_attribute('motion_blur_max_distortion', '0')
        camera_bp.set_attribute('motion_blur_min_object_screen_size', '0')
        camera_bp.set_attribute('shutter_speed', '1000') #1 ms shutter_speed
        camera_bp.set_attribute('lens_k', '0')
        camera_bp.set_attribute('lens_kcube', '0')
        camera_bp.set_attribute('lens_x_size', '0')
        camera_bp.set_attribute('lens_y_size', '0')
        return camera_bp
     
    def save(self):
        Camera.save(self)
        
        
class RGB_right(Camera):
    sensor_id_glob = 0

    def __init__(self, vehicle, world, actor_list, folder_output, transform):
        Camera.__init__(self, vehicle, world, actor_list, folder_output, transform)

    def set_attributes(self, blueprint_library):
        camera_bp = blueprint_library.find('sensor.camera.rgb')

        #camera_bp.set_attribute('image_size_x', '1392')
        #camera_bp.set_attribute('image_size_y', '1024')
        camera_bp.set_attribute('image_size_x', '696')
        camera_bp.set_attribute('image_size_y', '512')
        camera_bp.set_attribute('fov', '72') #72 degrees # Always fov on width even if width is different than height
        camera_bp.set_attribute('enable_postprocess_effects', 'True')
        camera_bp.set_attribute('sensor_tick', '0.10') # 10Hz camera
        camera_bp.set_attribute('gamma', '2.2')
        camera_bp.set_attribute('motion_blur_intensity', '0')
        camera_bp.set_attribute('motion_blur_max_distortion', '0')
        camera_bp.set_attribute('motion_blur_min_object_screen_size', '0')
        camera_bp.set_attribute('shutter_speed', '1000') #1 ms shutter_speed
        camera_bp.set_attribute('lens_k', '0')
        camera_bp.set_attribute('lens_kcube', '0')
        camera_bp.set_attribute('lens_x_size', '0')
        camera_bp.set_attribute('lens_y_size', '0')
        return camera_bp
     
    def save(self):
        Camera.save(self)

class IS(Camera):
    sensor_id_glob = 10
    def __init__(self, vehicle, world, actor_list, folder_output, transform):
        Camera.__init__(self, vehicle, world, actor_list, folder_output, transform)

    def set_attributes(self, blueprint_library):
        camera_is_bp = blueprint_library.find('sensor.camera.instance_segmentation')

        #camera_is_bp.set_attribute('image_size_x', '1392')
        #camera_is_bp.set_attribute('image_size_y', '1024')
        camera_is_bp.set_attribute('image_size_x', '696')
        camera_is_bp.set_attribute('image_size_y', '512')
        camera_is_bp.set_attribute('fov', '72') #72 degrees # Always fov on width even if width is different than height
        camera_is_bp.set_attribute('sensor_tick', '0.10') # 10Hz camera
        return camera_is_bp

    def save(self):
        Camera.save(self)


class Depth(Camera):
    sensor_id_glob = 20
    def __init__(self, vehicle, world, actor_list, folder_output, transform):
        Camera.__init__(self, vehicle, world, actor_list, folder_output, transform)

    def set_attributes(self, blueprint_library):
        camera_ss_bp = blueprint_library.find('sensor.camera.depth')

        #camera_ss_bp.set_attribute('image_size_x', '1392')
        #camera_ss_bp.set_attribute('image_size_y', '1024')
        
        camera_ss_bp.set_attribute('image_size_x', '696')
        camera_ss_bp.set_attribute('image_size_y', '512')
        camera_ss_bp.set_attribute('fov', '72') #72 degrees # Always fov on width even if width is different than height
        camera_ss_bp.set_attribute('sensor_tick', '0.10') # 10Hz camera
        return camera_ss_bp

    def save(self):
        cc = carla.ColorConverter.LogarithmicDepth
        Camera.save(self,cc)

class Original_Depth(Camera):
    sensor_id_glob = 20
    def __init__(self, vehicle, world, actor_list, folder_output, transform):
        Camera.__init__(self, vehicle, world, actor_list, folder_output, transform)

    def set_attributes(self, blueprint_library):
        camera_ss_bp = blueprint_library.find('sensor.camera.depth')

        #camera_ss_bp.set_attribute('image_size_x', '1392')
        #camera_ss_bp.set_attribute('image_size_y', '1024')
        
        camera_ss_bp.set_attribute('image_size_x', '696')
        camera_ss_bp.set_attribute('image_size_y', '512')
        camera_ss_bp.set_attribute('fov', '72') #72 degrees # Always fov on width even if width is different than height
        camera_ss_bp.set_attribute('sensor_tick', '0.10') # 10Hz camera
        return camera_ss_bp

    def save(self):
        Camera.save(self)
        
class Normal(Camera):
    sensor_id_glob = 20
    def __init__(self, vehicle, world, actor_list, folder_output, transform):
        Camera.__init__(self, vehicle, world, actor_list, folder_output, transform)

    def set_attributes(self, blueprint_library):
        # camera_ss_bp = blueprint_library.find('sensor.camera.normals')
        camera_ss_bp = blueprint_library.find('sensor.camera.depth')


        #camera_ss_bp.set_attribute('image_size_x', '1392')
        #camera_ss_bp.set_attribute('image_size_y', '1024')
        
        camera_ss_bp.set_attribute('image_size_x', '696')
        camera_ss_bp.set_attribute('image_size_y', '512')
        camera_ss_bp.set_attribute('fov', '72') #72 degrees # Always fov on width even if width is different than height
        camera_ss_bp.set_attribute('sensor_tick', '0.10') # 10Hz camera
        return camera_ss_bp

    def save(self):
        Camera.save(self)



class SS(Camera):
    sensor_id_glob = 10
    def __init__(self, vehicle, world, actor_list, folder_output, transform):
        Camera.__init__(self, vehicle, world, actor_list, folder_output, transform)

    def set_attributes(self, blueprint_library):
        camera_ss_bp = blueprint_library.find('sensor.camera.semantic_segmentation')

        #camera_ss_bp.set_attribute('image_size_x', '1392')
        #camera_ss_bp.set_attribute('image_size_y', '1024')
        
        camera_ss_bp.set_attribute('image_size_x', '696')
        camera_ss_bp.set_attribute('image_size_y', '512')
        camera_ss_bp.set_attribute('fov', '72') #72 degrees # Always fov on width even if width is different than height
        camera_ss_bp.set_attribute('sensor_tick', '0.10') # 10Hz camera
        return camera_ss_bp

    def save(self, color_converter=carla.ColorConverter.CityScapesPalette):
        Camera.save(self, color_converter)
    
class IMU(Camera):
    sensor_id_glob = 0
    def __init__(self, vehicle, world, actor_list, folder_output, transform):
        Camera.__init__(self, vehicle, world, actor_list, folder_output, transform)

    def set_attributes(self, blueprint_library):
        camera_IMU_bp = blueprint_library.find('sensor.other.imu')
        camera_IMU_bp.set_attribute('sensor_tick', '0.10') # 10Hz camera
        return camera_IMU_bp

    def save(self,vehicle,vehicles_list):
        while not self.queue.empty():
            data = self.queue.get()
            ts = data.timestamp-Sensor.initial_ts
            if ts - self.ts_tmp > 0.11 or (ts - self.ts_tmp) < 0: #check for 10Hz camera acquisition
                print("[Error in timestamp] Camera: previous_ts %f -> ts %f" %(self.ts_tmp, ts))
                # sys.exit()
            self.ts_tmp = ts
            with open(self.folder_output+"/data_imu/IMU.txt", 'a') as IMU :
                print("%05d"%(self.sensor_frame_id)+","+str(data.timestamp)+","+\
                    str(data.accelerometer.x)+","+str(data.accelerometer.y)+","+str(data.accelerometer.z)+","+\
                    str(data.transform.location.x)+","+str(data.transform.location.y)+","+str(data.transform.location.z)+","+\
                    str(data.gyroscope.x)+","+str(data.gyroscope.y)+","+str(data.gyroscope.z)+","+str(data.compass),file = IMU)
            IMU_path = self.frame_output+"/%05d" %(self.sensor_frame_id)
            print("Export : "+IMU_path)


            for x in vehicles_list:
                otherMatrix = carla.Transform.get_matrix(x.get_transform())
                location_callback(otherMatrix,self.queue)


            # data = self.queue.get()
            # ts = data.timestamp-Sensor.initial_ts
            # if ts - self.ts_tmp > 0.11 or (ts - self.ts_tmp) < 0: #check for 10Hz camera acquisition
            #     print("[Error in timestamp] Camera: previous_ts %f -> ts %f" %(self.ts_tmp, ts))
            #     sys.exit()
            # self.ts_tmp = ts
            with open(self.folder_output+"/Mycar_pose.txt", 'a') as fullpose:
                MycarMatrix = carla.Transform.get_matrix(vehicle.get_transform())
                print("%05d"%(self.sensor_frame_id)+","+(str(MycarMatrix[0][0]) + "," + str(MycarMatrix[0][1]) + "," + str(MycarMatrix[0][2]) + "," + str(MycarMatrix[0][3]) + "," +\
                    str(MycarMatrix[1][0]) + "," + str(MycarMatrix[1][1]) + "," + str(MycarMatrix[1][2]) + "," + str(MycarMatrix[1][3]) + "," +\
                    str(MycarMatrix[2][0]) + "," + str(MycarMatrix[2][1]) + "," + str(MycarMatrix[2][2]) + "," + str(MycarMatrix[2][3])),file=fullpose)
            print("Mycar location stored")


            if not os.path.exists(self.folder_output+"/location"):
                os.makedirs(self.folder_output+"/location")
            for i in range(0,len(vehicles_list)):
                dataL = self.queue.get()
                with open(self.folder_output+"/location"+"/%s.txt"%vehicles_list[i].id , 'a') as Vpose:
                    print("%05d"%(self.sensor_frame_id)+","+(str(dataL[0][0]) + "," + str(dataL[0][1]) + "," + str(dataL[0][2]) + "," + str(dataL[0][3]) + "," +\
                    str(dataL[1][0]) + "," + str(dataL[1][1]) + "," + str(dataL[1][2]) + "," + str(dataL[1][3]) + "," +\
                    str(dataL[2][0]) + "," + str(dataL[2][1]) + "," + str(dataL[2][2]) + "," + str(dataL[2][3])),file=Vpose)
            print("All vehicle's location stored")



            # if self.sensor_id == 0:
            #     with open(self.folder_output+"/Mycar_pose.txt", 'a') as fullpose:
            #         MycarMatrix = carla.Transform.get_matrix(vehicle.get_transform())
            #         print((str(MycarMatrix[0][0]) + "," + str(MycarMatrix[0][1]) + "," + str(MycarMatrix[0][2]) + "," + str(MycarMatrix[0][3]) + "," +\
            #             str(MycarMatrix[1][0]) + "," + str(MycarMatrix[1][1]) + "," + str(MycarMatrix[1][2]) + "," + str(MycarMatrix[1][3]) + "," +\
            #             str(MycarMatrix[2][0]) + "," + str(MycarMatrix[2][1]) + "," + str(MycarMatrix[2][2]) + "," + str(MycarMatrix[2][3])),file=fullpose)
            #         print("Mycar location stored")
            #     if not os.path.exists(self.folder_output+"/location"):
            #         os.makedirs(self.folder_output+"/location")
            #     for x in vehicles_list:
            #         otherMatrix = carla.Transform.get_matrix(x.get_transform())
            #         with open(self.folder_output +"/location" "/%s.txt"%x.semantic_tags , 'a') as Vpose:
            #             print((str(otherMatrix[0][0]) + "," + str(otherMatrix[0][1]) + "," + str(otherMatrix[0][2]) + "," + str(otherMatrix[0][3]) + "," +\
            #             str(otherMatrix[1][0]) + "," + str(otherMatrix[1][1]) + "," + str(otherMatrix[1][2]) + "," + str(otherMatrix[1][3]) + "," +\
            #             str(otherMatrix[2][0]) + "," + str(otherMatrix[2][1]) + "," + str(otherMatrix[2][2]) + "," + str(otherMatrix[2][3])),file=Vpose)
            #     print("All vehicle's location stored")
            #     with open(self.folder_output+"/full_ts_camera.txt", 'a') as file:
            #         file.write(str(self.sensor_frame_id)+" "+str(data.timestamp - Sensor.initial_ts)+"\n") #bug in CARLA 0.9.10: timestamp of camera is one tick late. 1 tick = 1/fps_simu seconds
            self.sensor_frame_id += 1

class Optical(Camera):
    sensor_id_glob = 10
    def __init__(self, vehicle, world, actor_list, folder_output, transform):
        Camera.__init__(self, vehicle, world, actor_list, folder_output, transform)

    def set_attributes(self, blueprint_library):
        #add a optical flow camera
        camera_op_bp = blueprint_library.find('sensor.camera.optical_flow')
        #camera_op_bp.set_attribute('image_size_x', '1392')
        #camera_op_bp.set_attribute('image_size_y', '1024')
        
        camera_op_bp.set_attribute('image_size_x', '696')
        camera_op_bp.set_attribute('image_size_y', '512')
        camera_op_bp.set_attribute('sensor_tick', '0.10') # 10Hz camera
        return camera_op_bp
    
    def save(self):
        while not self.queue.empty():
            data = self.queue.get()
            ts = data.timestamp-Sensor.initial_ts
            if ts - self.ts_tmp > 0.11 or (ts - self.ts_tmp) < 0: #check for 10Hz camera acquisition
                print("[Error in timestamp] Camera: previous_ts %f -> ts %f" %(self.ts_tmp, ts))
                # sys.exit()
            self.ts_tmp = ts
            image = data.get_color_coded_flow()
            buffer = np.frombuffer(image.raw_data, dtype=np.uint8)
            buffer = buffer.reshape(image.height, image.width, 4)[..., [2, 1, 0]]  # BGRA -> RGB
            opt_path = self.frame_output+"/%05d.png" %(self.sensor_frame_id)
            Image.fromarray(buffer).save(opt_path)
            print("Export : "+opt_path)
            if self.sensor_id == 0:
                with open(self.folder_output+"/full_ts_camera.txt", 'a') as file:
                    file.write(str(self.sensor_frame_id)+" "+str(data.timestamp - Sensor.initial_ts)+"\n") #bug in CARLA 0.9.13: timestamp of camera is one tick late. 1 tick = 1/fps_simu seconds
            self.sensor_frame_id += 1
         
            # buffer = np.frombuffer(data.raw_data, dtype=np.uint8)
            # buffer = np.reshape(buffer,(data.height, data.width, 8))
            # buffer = buffer[:,:,:3]
            # cv2.imwrite(self.frame_output+"/%05d.png" %(self.sensor_frame_id),buffer)
    
class HDL64E(Sensor):
    sensor_id_glob = 100
    def __init__(self, vehicle, world, actor_list, folder_output, transform):
        self.rotation_lidar = rotation_carla(transform.rotation)
        self.rotation_lidar_transpose = self.rotation_lidar.T
        self.queue = queue.PriorityQueue()
        self.bp = self.set_attributes(world.get_blueprint_library())
        self.sensor = world.spawn_actor(self.bp, transform, attach_to=vehicle)
        actor_list.append(self.sensor)
        self.sensor.listen(lambda data: self.queue.put((data.timestamp, data)))
        self.sensor_id = self.__class__.sensor_id_glob;
        self.__class__.sensor_id_glob += 1
        self.folder_output = folder_output
        
        self.i_packet = 0
        self.i_frame = 0

        self.initial_loc = np.zeros(3)
        self.initial_rot = np.identity(3)
        self.calib_output = folder_output
        self.frame_output = folder_output+"/frames"
        os.makedirs(self.frame_output) if not os.path.exists(self.frame_output) else [os.remove(f) for f in glob.glob(self.frame_output+"/*") if os.path.isfile(f)]

        settings = world.get_settings()
        self.packet_per_frame = 1/(self.bp.get_attribute('rotation_frequency').as_float()*settings.fixed_delta_seconds)
        self.packet_period = settings.fixed_delta_seconds

        self.pt_size = 4*4 #(4 float32 with x, y, z, timestamp)

        header = ['ply']
        header.append('format binary_little_endian 1.0')
        header.append('element vertex ')
        self.begin_header = '\n'.join(header)
        header = ["property float x"]
        header.append("property float y")
        header.append("property float z")
        header.append("property float timestamp")
        header.append('end_header')
        self.end_header = '\n'.join(header)+'\n'

        self.list_pts = []
        self.list_semantic = []
        self.list_ts = []
        self.list_trajectory = []

        self.ts_tmp = 0.0
        print('created %s' % self.sensor)

    # def init(self):
    #     self.initial_loc = translation_carla(self.sensor.get_location())
    #     self.initial_rot_transpose = (rotation_carla(self.sensor.get_transform().rotation).dot(self.rotation_lidar)).T
    #     with open(self.calib_output+"/full_poses_lidar.txt", 'w') as posfile:
    #         posfile.write("# R(0,0) R(0,1) R(0,2) t(0) R(1,0) R(1,1) R(1,2) t(1) R(2,0) R(2,1) R(2,2) t(2) timestamp\n")

    def save(self):
        while not self.queue.empty():
            data = self.queue.get()[1]
            ts = data.timestamp-Sensor.initial_ts
            if ts-self.ts_tmp > self.packet_period*1.5 or ts-self.ts_tmp < 0:
                print("[Error in timestamp] HDL64E: previous_ts %f -> ts %f" %(self.ts_tmp, ts))
                # sys.exit()

            self.ts_tmp = ts

            nbr_pts = len(data.raw_data)//24 #4 float32 and 2 uint
            self.list_ts.append(np.broadcast_to(ts, nbr_pts))
            buffer = np.frombuffer(data.raw_data, dtype=np.dtype([('x','f4'),('y','f4'),('z','f4'),('cos','f4'),('index','u4'),('semantic','u4')]))
            
            # We're negating the y to correctly visualize a world that matches what we see in Unreal since we uses a right-handed coordinate system
            self.list_pts.append(np.array([buffer[:]['x'], -buffer[:]['y'], buffer[:]['z'], buffer[:]['cos']]))
            self.list_semantic.append(np.array([buffer[:]['index'], buffer[:]['semantic']]))
            
            self.i_packet += 1
            if self.i_packet%self.packet_per_frame == 0:
                pts_all = np.hstack(self.list_pts)
                pts_all[0:3,:] = self.rotation_lidar_transpose.dot(pts_all[0:3,:])
                pts_all = pts_all.T
                # semantic_all = np.hstack(self.list_semantic).T
                # ts_all = np.concatenate(self.list_ts)
                self.list_pts = []
                self.list_semantic = []
                self.list_ts = []

                # ply_file_path = self.frame_output+"/frame_%05d.ply" %self.i_frame

                # if ply.write_ply(ply_file_path, [np.float32(pts_all), np.float32(ts_all), np.uint32(semantic_all)], ['x','y','z','cos_angle_lidar_surface','timestamp','instance','semantic']):
                #     print("Export : "+ply_file_path)
                # else:
                #     print('ply.write_ply() failed')

                self.i_frame += 1

            # R_W = self.initial_rot_transpose.dot(rotation_carla(data.transform.rotation).dot(self.rotation_lidar))
            # T_W = self.initial_rot_transpose.dot(translation_carla(data.transform.location) - self.initial_loc)

            # with open(self.calib_output+"/full_poses_lidar.txt", 'a') as posfile:
            #     posfile.write(" ".join(map(str,[r for r in R_W[0]]))+" "+str(T_W[0])+" ")
            #     posfile.write(" ".join(map(str,[r for r in R_W[1]]))+" "+str(T_W[1])+" ")
            #     posfile.write(" ".join(map(str,[r for r in R_W[2]]))+" "+str(T_W[2])+" ")
            #     posfile.write(str(ts)+"\n")

            # self.list_trajectory.extend([struct.pack('f', T) for T in T_W])
            # self.list_trajectory.append(struct.pack('f', ts))

        return self.i_frame
        
    # def save_poses(self):
    #     ply_file_path = self.calib_output+"/poses_lidar.ply"
    #     trajectory_bytes = b''.join(self.list_trajectory)
    #     with open(ply_file_path, 'w') as posfile:
    #         posfile.write(self.begin_header+str(len(trajectory_bytes)//self.pt_size)+'\n'+self.end_header)
    #     with open(ply_file_path, 'ab') as posfile:
    #         posfile.write(trajectory_bytes)
    #     print("Export : "+ply_file_path)
        

    def set_attributes(self, blueprint_library):
        lidar_bp = blueprint_library.find('sensor.lidar.ray_cast_semantic')
        lidar_bp.set_attribute('channels', '64')
        lidar_bp.set_attribute('range', '80.0')    # 80.0 m
        lidar_bp.set_attribute('points_per_second', str(64/0.00004608))
        lidar_bp.set_attribute('rotation_frequency', '10')
        lidar_bp.set_attribute('upper_fov', str(2))
        lidar_bp.set_attribute('lower_fov', str(-24.8))
        return lidar_bp

# Function to change rotations in CARLA from left-handed to right-handed reference frame
def rotation_carla(rotation):
    cr = math.cos(math.radians(rotation.roll))
    sr = math.sin(math.radians(rotation.roll))
    cp = math.cos(math.radians(rotation.pitch))
    sp = math.sin(math.radians(rotation.pitch))
    cy = math.cos(math.radians(rotation.yaw))
    sy = math.sin(math.radians(rotation.yaw))
    return np.array([[cy*cp, -cy*sp*sr+sy*cr, -cy*sp*cr-sy*sr],[-sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],[sp, cp*sr, cp*cr]])

# Function to change translations in CARLA from left-handed to right-handed reference frame
def translation_carla(location):
    if isinstance(location, np.ndarray):
        return location*(np.array([[1],[-1],[1]]))
    else:
        return np.array([location.x, -location.y, location.z])

def transform_lidar_to_camera(lidar_tranform, camera_transform):
    R_camera_vehicle = rotation_carla(camera_transform.rotation)
    R_lidar_vehicle = np.identity(3) #rotation_carla(lidar_tranform.rotation) #we want the lidar frame to have x forward
    R_lidar_camera = R_camera_vehicle.T.dot(R_lidar_vehicle)
    T_lidar_camera = R_camera_vehicle.T.dot(translation_carla(np.array([[lidar_tranform.location.x],[lidar_tranform.location.y],[lidar_tranform.location.z]])-np.array([[camera_transform.location.x],[camera_transform.location.y],[camera_transform.location.z]])))
    return np.vstack((np.hstack((R_lidar_camera, T_lidar_camera)), [0,0,0,1]))

# def screenshot(vehicle, world, actor_list, folder_output, transform):
#     sensor = world.spawn_actor(RGB.set_attributes(RGB, world.get_blueprint_library()), transform, attach_to=vehicle)
#     actor_list.append(sensor)
#     screenshot_queue = queue.Queue()
#     sensor.listen(screenshot_queue.put)
#     print('created %s' % sensor)

#     while screenshot_queue.empty(): world.tick()

#     file_path = folder_output+"/screenshot.png"
#     screenshot_queue.get().save_to_disk(file_path)
#     print("Export : "+file_path)
#     actor_list[-1].destroy()
#     print('destroyed %s' % actor_list[-1])
#     del actor_list[-1]


            
def spawn_npc(client, nbr_vehicles, nbr_walkers, vehicles_list, all_walkers_id):
        client.set_timeout(2.0)
        world = client.get_world()

        traffic_manager = client.get_trafficmanager(5000)

        
        #traffic_manager.set_hybrid_physics_mode(True)
        #traffic_manager.set_random_device_seed(args.seed)

        traffic_manager.set_synchronous_mode(True)
        synchronous_master = True

        blueprints = world.get_blueprint_library().filter('vehicle.*')
        #blueprints = world.get_blueprint_library().filter('vehicle.carlamotors.firetruck')
        # blueprints += world.get_blueprint_library().filter('vehicle.tesla.cybertruck')
        
        blueprintsWalkers = world.get_blueprint_library().filter('walker.pedestrian.*')

        safe = True
        if safe:
                blueprints = [x for x in blueprints if int(x.get_attribute('number_of_wheels')) == 4]
                blueprints = [x for x in blueprints if not x.id.endswith('isetta')]
                blueprints = [x for x in blueprints if not x.id.endswith('carlacola')]
                blueprints = [x for x in blueprints if not x.id.endswith('cybertruck')]
                blueprints = [x for x in blueprints if not x.id.endswith('t2')]

        blueprints = sorted(blueprints, key=lambda bp: bp.id)

        spawn_points = world.get_map().get_spawn_points()
        
        center = carla.Location(x=127, y= 195, z=0)
        # care: my sapwn point of ego is actually 127.4,-195.2,2, but in carla y is = -y 
        radius = 200
        # 过滤出在圆内的生成点
        spawn_points = [sp for sp in spawn_points if sp.location.distance(center) < radius]
        
        """
        spawn_point = carla.Transform(carla.Location(x=120.4, y=199.2, z=0.5))
        # 在指定的坐标上生成车辆
        vehicle_bp = random.choice(blueprints)
        vehicle = world.spawn_actor(vehicle_bp, spawn_point)
        vehicle.set_autopilot(True, traffic_manager.get_port())
        """

        number_of_spawn_points = len(spawn_points)
        print("Number of spawn points : ", number_of_spawn_points)

        if nbr_vehicles <= number_of_spawn_points:
                random.shuffle(spawn_points)
        elif nbr_vehicles > number_of_spawn_points:
                msg = 'requested %d vehicles, but could only find %d spawn points'
                logging.warning(msg, nbr_vehicles, number_of_spawn_points)
                nbr_vehicles = number_of_spawn_points

        # @todo cannot import these directly.
        SpawnActor = carla.command.SpawnActor 
        SetAutopilot = carla.command.SetAutopilot  
        SetVehicleLightState = carla.command.SetVehicleLightState
        FutureActor = carla.command.FutureActor

        # --------------
        # Spawn vehicles
        # --------------
        
        for n, transform in enumerate(spawn_points):
                if n >= nbr_vehicles:
                        break
                blueprint = random.choice(blueprints)
                if blueprint.has_attribute('color'):
                        color = random.choice(blueprint.get_attribute('color').recommended_values)
                        blueprint.set_attribute('color', color)
                if blueprint.has_attribute('driver_id'):
                        driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
                        blueprint.set_attribute('driver_id', driver_id)
                blueprint.set_attribute('role_name', 'autopilot')

                # prepare the light state of the cars to spawn
                light_state = vls.NONE
                car_lights_on = False
                if car_lights_on:
                        light_state = vls.Position | vls.LowBeam | vls.LowBeam
                """
                # spawn the cars and set their autopilot and light state all together
                batch.append(SpawnActor(blueprint, transform)
                        .then(SetAutopilot(FutureActor, True, traffic_manager.get_port()))
                        .then(SetVehicleLightState(FutureActor, light_state)))
                """

        """
        for response in client.apply_batch_sync(batch, synchronous_master):
                if response.error:
                        logging.error(response.error)
                else:
                        vehicles_list.append(response.actor_id)
                        '''
                        vehicle = world.get_actor(response.actor_id)
                        if vehicle.is_autopilot_enabled:
                            print(f"Vehicle {vehicle.id} is connected to traffic_manager.")
                        else:
                            print(f"Vehicle {vehicle.id} is not connected to traffic_manager.")
                        '''
        """
        
        for i in range(nbr_vehicles):
            # 选择一个生成点
            spawn_point = random.choice(spawn_points)
            spawn_points.remove(spawn_point) #remove after using
            vehicle_bp = random.choice(blueprints)
            vehicle = world.spawn_actor(vehicle_bp, spawn_point)
            vehicle.set_autopilot(True, traffic_manager.get_port())  #autopilot
            speed_kmh = 40  
            #traffic_manager.set_desired_speed(vehicle, speed_kmh / 3.6)  # m/s
            vehicles_list.append(vehicle)
            print(i,"npc established!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            
        traffic_manager.set_hybrid_physics_mode(enabled=True)
        traffic_manager.set_hybrid_physics_radius(r=100.0)
        traffic_manager.global_percentage_speed_difference(-20)

        traffic_manager.set_global_distance_to_leading_vehicle(3.0)   

        # world.tick()  # Update world state here
        return vehicles_list
        


        # -------------
        # Spawn Walkers
        # -------------
        # some settings
        # walkers_list = []
        # percentagePedestriansRunning = 0.0            # how many pedestrians will run
        # percentagePedestriansCrossing = 0.0         # how many pedestrians will walk through the road
        # # 1. take all the random locations to spawn
        # spawn_points = []
        # all_loc = []
        # i = 0
        # while i < nbr_walkers:
        #         spawn_point = carla.Transform()
        #         loc = world.get_random_location_from_navigation()
        #         if ((loc != None) and not(loc in all_loc)):
        #                 spawn_point.location = loc
        #                 spawn_points.append(spawn_point)
        #                 all_loc.append(loc)
        #                 i = i + 1
        # # 2. we spawn the walker object
        # batch2 = []
        # walker_speed = []
        # for spawn_point in spawn_points:
        #         walker_bp = random.choice(blueprintsWalkers)
        #         # set as not invincible
        #         if walker_bp.has_attribute('is_invincible'):
        #                 walker_bp.set_attribute('is_invincible', 'false')
        #         # set the max speed
        #         if walker_bp.has_attribute('speed'):
        #                 if (random.random() > percentagePedestriansRunning):
        #                         # walking
        #                         walker_speed.append(walker_bp.get_attribute('speed').recommended_values[1])
        #                 else:
        #                         # running
        #                         walker_speed.append(walker_bp.get_attribute('speed').recommended_values[2])
        #         else:
        #                 print("Walker has no speed")
        #                 walker_speed.append(0.0)
        #         batch2.append(SpawnActor(walker_bp, spawn_point))
        # results = client.apply_batch_sync(batch2, True)
        # walker_speed2 = []
        # for i in range(len(results)):
        #         if results[i].error:
        #                 logging.error(results[i].error)
        #         else:
        #                 walkers_list.append({"id": results[i].actor_id})
        #                 walker_speed2.append(walker_speed[i])
        # walker_speed = walker_speed2
        # # 3. we spawn the walker controller
        # batch3 = []
        # walker_controller_bp = world.get_blueprint_library().find('controller.ai.walker')
        # for i in range(len(walkers_list)):
        #         batch3.append(SpawnActor(walker_controller_bp, carla.Transform(), walkers_list[i]["id"]))
        # results = client.apply_batch_sync(batch3, True)
        # for i in range(len(results)):
        #         if results[i].error:
        #                 logging.error(results[i].error)
        #         else:
        #                 walkers_list[i]["con"] = results[i].actor_id
        # # 4. we put altogether the walkers and controllers id to get the objects from their id
        # for i in range(len(walkers_list)):
        #         all_walkers_id.append(walkers_list[i]["con"])
        #         all_walkers_id.append(walkers_list[i]["id"])
        # all_actors = world.get_actors(all_walkers_id)

        # # wait for a tick to ensure client receives the last transform of the walkers we have just created
        # world.tick()

        # # 5. initialize each controller and set target to walk to (list is [controler, actor, controller, actor ...])
        # # set how many pedestrians can cross the road
        # world.set_pedestrians_cross_factor(percentagePedestriansCrossing)
        # for i in range(0, len(all_walkers_id), 2):
        #         # start walker
        #         all_actors[i].start()
        #         # set walk to random point
        #         all_actors[i].go_to_location(world.get_random_location_from_navigation())
        #         # max speed
        #         all_actors[i].set_max_speed(float(walker_speed[int(i/2)]))

        print('Spawned %d vehicles and 0 walkers' % (len(batch)))
        # example of how to use parameters
        traffic_manager.global_percentage_speed_difference(30.0)

def follow(transform, world):    # Transforme carla.Location(x,y,z) from sensor to world frame
    rot = transform.rotation
    rot.pitch = -25 
    world.get_spectator().set_transform(carla.Transform(transform.transform(carla.Location(x=-15,y=0,z=5)), rot))


