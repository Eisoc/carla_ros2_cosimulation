#!/usr/bin/env python
#
# Copyright (c) 2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
"""
Generates a plan of waypoints to follow

It uses the current pose of the ego vehicle as starting point. If the
vehicle is respawned or move, the route is newly calculated.

The goal is either read from the ROS topic `/carla/<ROLE NAME>/move_base_simple/goal`, if available
(e.g. published by RVIZ via '2D Nav Goal') or a fixed point is used.

The calculated route is published on '/carla/<ROLE NAME>/waypoints'

Additionally, services are provided to interface CARLA waypoints.
"""
import math
import sys
import threading

import carla
from agents.navigation.global_route_planner import GlobalRoutePlanner

import carla_common.transforms as trans
import ros_compatibility as roscomp
from ros_compatibility.exceptions import *
from ros_compatibility.node import CompatibleNode
from ros_compatibility.qos import QoSProfile, DurabilityPolicy

from carla_msgs.msg import CarlaWorldInfo
from carla_waypoint_types.srv import GetWaypoint, GetActorWaypoint
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

# import about kitti
import glob
import os
# from pathlib import Path as Path2
# import random
current_file_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(current_file_path)

try:
    '''
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
    '''
    sys.path.append("/home/bing/carla-ros-bridge/src/ros-bridge/carla_ad_demo/launch/")
    sys.path.append("/home/bing/carla-ros-bridge/src/ros-bridge/install/carla_waypoint_publisher/lib/python3.8/site-packages/carla_waypoint_publisher/")

except IndexError:
    pass

import time
from datetime import date
import generator_KITTI as gen
import traceback

#

class CarlaToRosWaypointConverter(CompatibleNode):

    """
    This class generates a plan of waypoints to follow.

    The calculation is done whenever:
    - the hero vehicle appears
    - a new goal is set
    """
    WAYPOINT_DISTANCE = 2.0

    def __init__(self):
        """
        Constructor
        """
        super(CarlaToRosWaypointConverter, self).__init__('carla_waypoint_publisher')
        # -M def global var client
        self.carla_client = None
        self.world = None
        
        # threading
        self.thread = threading.Thread(target=self.KITTI_gen)

        
        # init sensors
        self.first_call = True
        self.VelodyneHDL64 = None
        self.RGB_left = None
        self.RGB_right = None
        self.ins_segmc = None
        # depth = gen.Depth(MyCar, world, actor_list, folder_output, right_transform)
        self.originDepth = None
        self.normals = None
        self.optical = None
        self.IMU = None
        self.SemSeg = None
        
        
        self.connect_to_carla()
        self.map = self.world.get_map()
        self.ego_vehicle = None
        self.ego_vehicle_location = None
        self.on_tick = None
        self.role_name = self.get_param("role_name", 'ego_vehicle')
        self.waypoint_publisher = self.new_publisher(
            Path,
            '/carla/{}/waypoints'.format(self.role_name),
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

        # initialize ros services
        self.get_waypoint_service = self.new_service(
            GetWaypoint,
            '/carla_waypoint_publisher/{}/get_waypoint'.format(self.role_name),
            self.get_waypoint)
        self.get_actor_waypoint_service = self.new_service(
            GetActorWaypoint,
            '/carla_waypoint_publisher/{}/get_actor_waypoint'.format(self.role_name),
            self.get_actor_waypoint)


        # set initial goal

        # self.goal = self.world.get_map().get_spawn_points()[0]
        # I have set the initial goal to None, then the car will at state "stop" wenn spawned
        self.goal = None

        self.current_route = None
        self.goal_subscriber = self.new_subscription(
            PoseStamped,
            # M- "/carla/{}/goal".format(self.role_name),
            '/move_base_simple/goal',
            self.on_goal,
            qos_profile=10)

        # use callback to wait for ego vehicle
        self.loginfo("Waiting for ego vehicle...")
        self.on_tick = self.world.on_tick(self.find_ego_vehicle_actor)
        

        
        # M- subscribe vehicle info in carla
        # self.vehicle_info_sub = rospy.Subscriber('/carla/ego_vehicle/vehicle_info', Car   laEgoVehicleInfo, self.vehicle_info_callback)
        
        

    def destroy(self):
        """
        Destructor
        """
        self.ego_vehicle = None
        if self.on_tick:
            self.world.remove_on_tick(self.on_tick)

    def get_waypoint(self, req, response=None):
        """
        Get the waypoint for a location
        """
        carla_position = carla.Location()
        carla_position.x = req.location.x
        carla_position.y = -req.location.y
        carla_position.z = req.location.z

        carla_waypoint = self.map.get_waypoint(carla_position)

        response = roscomp.get_service_response(GetWaypoint)
        response.waypoint.pose = trans.carla_transform_to_ros_pose(carla_waypoint.transform)
        response.waypoint.is_junction = carla_waypoint.is_junction
        response.waypoint.road_id = carla_waypoint.road_id
        response.waypoint.section_id = carla_waypoint.section_id
        response.waypoint.lane_id = carla_waypoint.lane_id
        return response

    def get_actor_waypoint(self, req, response=None):
        """
        Convenience method to get the waypoint for an actor
        """
        # self.loginfo("get_actor_waypoint(): Get waypoint of actor {}".format(req.id))
        actor = self.world.get_actors().find(req.id)

        response = roscomp.get_service_response(GetActorWaypoint)
        if actor:
            carla_waypoint = self.map.get_waypoint(actor.get_location())
            response.waypoint.pose = trans.carla_transform_to_ros_pose(carla_waypoint.transform)
            response.waypoint.is_junction = carla_waypoint.is_junction
            response.waypoint.road_id = carla_waypoint.road_id
            response.waypoint.section_id = carla_waypoint.section_id
            response.waypoint.lane_id = carla_waypoint.lane_id
        else:
            self.logwarn("get_actor_waypoint(): Actor {} not valid.".format(req.id))
        return response

    def on_goal(self, goal):
        """
        Callback for /move_base_simple/goal

        Receiving a goal (e.g. from RVIZ '2D Nav Goal') triggers a new route calculation.

        :return:
        """
        self.loginfo("Received goal, trigger rerouting...")
        carla_goal = trans.ros_pose_to_carla_transform(goal.pose)
        self.goal = carla_goal
        self.reroute()

    def reroute(self):
        """
        Triggers a rerouting
        """
        if self.ego_vehicle is None or self.goal is None:
            # no ego vehicle, remove route if published
            self.current_route = None
            self.publish_waypoints()
        else:
            self.current_route = self.calculate_route(self.goal)
        self.publish_waypoints()

    def find_ego_vehicle_actor(self, _):
        """
        Look for an carla actor with name 'ego_vehicle'
        """
        hero = None
        for actor in self.world.get_actors():
            if actor.attributes.get('role_name') == self.role_name:
                hero = actor
                break

        ego_vehicle_changed = False
        if hero is None and self.ego_vehicle is not None:
            ego_vehicle_changed = True

        if not ego_vehicle_changed and hero is not None and self.ego_vehicle is None:
            ego_vehicle_changed = True

        if not ego_vehicle_changed and hero is not None and \
                self.ego_vehicle is not None and hero.id != self.ego_vehicle.id:
            ego_vehicle_changed = True

        if ego_vehicle_changed:
            self.loginfo("Ego vehicle changed.")
            self.ego_vehicle = hero
            self.reroute()
        elif self.ego_vehicle:
            current_location = self.ego_vehicle.get_location()
            if self.ego_vehicle_location:
                dx = self.ego_vehicle_location.x - current_location.x
                dy = self.ego_vehicle_location.y - current_location.y
                distance = math.sqrt(dx * dx + dy * dy)
                if distance > self.WAYPOINT_DISTANCE:
                    self.loginfo("Ego vehicle was repositioned.")
                    self.reroute()
            self.ego_vehicle_location = current_location

    def calculate_route(self, goal):
        """
        Calculate a route from the current location to 'goal'
        """
        self.loginfo("Calculating route to x={}, y={}, z={}".format(
            goal.location.x,
            goal.location.y,
            goal.location.z))

        grp = GlobalRoutePlanner(self.world.get_map(), sampling_resolution=1)
        route = grp.trace_route(self.ego_vehicle.get_location(),
                                carla.Location(goal.location.x,
                                               goal.location.y,
                                               goal.location.z))

        return route

    def publish_waypoints(self):
        """
        Publish the ROS message containing the waypoints
        """
        msg = Path()
        msg.header.frame_id = "map"
        msg.header.stamp = roscomp.ros_timestamp(self.get_time(), from_sec=True)
        if self.current_route is not None:
            for wp in self.current_route:
                pose = PoseStamped()
                pose.pose = trans.carla_transform_to_ros_pose(wp[0].transform)
                msg.poses.append(pose)

        self.waypoint_publisher.publish(msg)
        self.loginfo("Published {} waypoints.".format(len(msg.poses)))
        time.sleep(1)
        print("goal changed, prepare to next record.")
        self.thread = threading.Thread(target=self.KITTI_gen)
        print("thread reset succeessfully.")
        self.thread.start()

    def connect_to_carla(self):

        self.loginfo("Waiting for CARLA world (topic: /carla/world_info)...")
        try:
            self.wait_for_message(
                "/carla/world_info",
                CarlaWorldInfo,
                qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
                timeout=15.0)
        except ROSException as e:
            self.logerr("Error while waiting for world info: {}".format(e))
            raise e

        host = self.get_param("host", "127.0.0.1")
        port = self.get_param("port", 2000)
        timeout = self.get_param("timeout", 17)
        self.loginfo("CARLA world available. Trying to connect to {host}:{port}".format(
            host=host, port=port))

        # carla_client = carla.Client(host=host, port=port)
        # carla_client.set_timeout(timeout)
        # M- use global class var client
        self.carla_client = carla.Client(host=host, port=port)
        self.carla_client.set_timeout(timeout)

        try:
            # M- also, var client
            self.world = self.carla_client.get_world()
        except RuntimeError as e:
            self.logerr("Error while connecting to Carla: {}".format(e))
            raise e

        self.loginfo("Connected to Carla.")
        settings = self.world.get_settings()
        
        
        
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.02
        settings.no_rendering_mode = False
        

        self.world.apply_settings(settings)
    # M-
    def vehicle_info_callback(self, msg):
        self.vehicle_id = msg.id  # 存储车辆的ID供后续使用
        
    def KITTI_gen(self):
    ############# NEW about KITTI
        # M- establish the vars and sensors, only once
  
        start_record_full = time.time()
        time_stop = 2.0
        nbr_frame = 1000 #MAX = 100000
        nbr_walkers = 0
        nbr_vehicles = 30

        actor_list = []
        vehicles_id_list = []
        all_walkers_id = []
        vehicles_list = []
    
        init_settings = None
        print("start KITTI generator")
        # M- def new class to generate data KITTI
        # generator = CarlaToRosWaypointConverter()
        
        # M-
        # client = carla.Client('localhost', 2000)

        client = self.carla_client
        
        init_settings = carla.WorldSettings()
        # client.set_timeout(100.0)
        print("Set our map!!")
        # print(client.get_available_maps())

        # world = client.get_world()
        world = self.world
        if not self.carla_client:
            raise RuntimeError("CARLA client has not been set")
            
        if not self.world:
            raise RuntimeError("Failed to get world from CARLA client")
        
        print("CARLA client:", self.carla_client)
        print("CARLA world:", self.world)
        
        # world = client.load_world("Town10HD")
        # folder_output = "/media/stuf/data/Dataset_BrandNew%s/%s/generated" %(client.get_client_version(), world.get_map().name)
        folder_output = "//home/bing/Pictures/generator/Dataset_BrandNew%s/%s/generated" %(client.get_client_version(), world.get_map().name)
        os.makedirs(folder_output) if not os.path.exists(folder_output) else [os.remove(f) for f in glob.glob(folder_output+"/*") if os.path.isfile(f)]
        client.start_recorder(os.path.dirname(os.path.realpath(__file__))+"/"+folder_output+"/recording.log")
    
        # Weather
        world.set_weather(carla.WeatherParameters.ClearNoon)
        
        traffic_lights = world.get_actors().filter('traffic.traffic_light')
        
        def set_traffic_light_to_green(traffic_light):
            if traffic_light.get_state() != carla.TrafficLightState.Green:
                traffic_light.set_state(carla.TrafficLightState.Green)
                traffic_light.set_green_time(1000)  # 设置绿灯持续时间（以秒为单位）

        for traffic_light in traffic_lights:
            set_traffic_light_to_green(traffic_light)
            print("Traffic light state set successfully!")
        # weather = carla.WeatherParameters(
        #     fog_density = 80.0,
        #     fog_distance = 1.0,
        #     # sun_azimuth_angle = 90.0,
        #     sun_altitude_angle = -90.0

        # )
        # world.set_weather(weather)

        # #Freeze all traffic lights
        # carla.TrafficLight.freeze(True)


        # Set Synchronous mode
        
        '''
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.01
        settings.no_rendering_mode = False

        world.apply_settings(settings)
        '''
            
        
        '''
        # Create our vehicle
        blueprint_librardescribe_sub_entitiesy = world.get_blueprint_library()
        bp_MyCar = blueprint_library.find('vehicle.lincoln.mkz_2020')
        bp_MyCar.set_attribute('color', '228, 239, 241')
        bp_MyCar.set_attribute('role_name', 'MyCar')

        #Our vehicle's start pose
        start_pose = random.choice(world.get_map().get_spawn_points())#
        
        start_pose_str = '127.4, -195.4, 2, 0, 0, 180'
        start_pose_values = [float(x) for x in start_pose_str.split(',')]

        location = carla.Location(x=start_pose_values[0], y=start_pose_values[1], z=start_pose_values[2])
        rotation = carla.Rotation(pitch=start_pose_values[3], yaw=start_pose_values[4], roll=start_pose_values[5])
        start_pose = carla.Transform(location, rotation)
        '''
        '''
        def find_ego_vehicle_actor():
            """
            Look for an carla actor with name 'ego_vehicle'
            """
            hero = None
            for actor in world.get_actors():
                if actor.attributes.get('role_name') == 'ego_vehicle':
                    hero = actor
                    break

            return hero
        '''
        # MyCar = find_ego_vehicle_actor()
        MyCar = self.ego_vehicle
        actor_list.append(MyCar)
        print('Created %s' % MyCar)

        # Spawn vehicles and walkers


        #Set instance id
        # world.set_instance_tagging_style("actor_id")

        '''
        # Wait for MyCar to stop
        start = world.get_snapshot().timestamp.elapsed_seconds
        print("Waiting for MyCar to stop ...")
        while world.get_snapshot().timestamp.elapsed_seconds-start < time_stop: world.tick()
        print("MyCar stopped")
        '''
        
        # Set sensors transformation from MyCar
        lidar_transform     = carla.Transform(carla.Location(x=0, y=0, z=1.80))
        left_transform = carla.Transform(carla.Location(x=2.0, y=-0.2, z=1.4))
        right_transform = carla.Transform(carla.Location(x=2.0, y=0.2, z=1.4))
        with open(folder_output+"/start_pose.txt", 'a') as pose :
            print("left camera pose: " + str(left_transform)+ "right camera pose: " + str(right_transform),file=pose)

        # Take a screenshot#We dont need this
        #gen.screenshot(MyCar, world, actor_list, folder_output, carla.Transform(carla.Location(x=0.0, y=0, z=2.0), carla.Rotation(pitch=0, yaw=0, roll=0)))

        # Create our sensors
        # M- IS and optical flow not supported
        if self.first_call == True:
            
            self.first_call = False
            gen.RGB_left.sensor_id_glob = 0
            gen.RGB_right.sensor_id_glob = 0
            gen.IS.sensor_id_glob = 10
            gen.Depth.sensor_id_glob = 20
            gen.HDL64E.sensor_id_glob = 100
            gen.IMU.sensor_id_glob = 0
            gen.Optical.sensor_id_glob = 0

            self.VelodyneHDL64 = gen.HDL64E(MyCar, world, actor_list, folder_output, lidar_transform)
            self.RGB_left = gen.RGB_left(MyCar, world, actor_list, folder_output, left_transform)
            self.RGB_right = gen.RGB_right(MyCar, world, actor_list, folder_output, right_transform)
            self.ins_segmc = gen.IS(MyCar, world, actor_list, folder_output, right_transform)
            # depth = gen.Depth(MyCar, world, actor_list, folder_output, right_transform)
            self.originDepth = gen.Original_Depth(MyCar, world, actor_list, folder_output, right_transform)
            self.normals = gen.Normal(MyCar, world, actor_list, folder_output, right_transform)
            self.optical = gen.Optical(MyCar, world, actor_list, folder_output, right_transform)
            self.IMU = gen.IMU(MyCar, world, actor_list, folder_output, right_transform)
            print("sensors. imu and ss,last test!")
            self.SemSeg = gen.SS(MyCar, world, actor_list, folder_output, right_transform)
    
            print("sensors established, first call finished")
            vehicles_id_list = gen.spawn_npc(client, nbr_vehicles, nbr_walkers, vehicles_id_list, all_walkers_id)
            print(vehicles_id_list)
            print("spawn points finished!")
            for x in vehicles_id_list:
                vehicles_list.append(world.get_actor(x))
                #must spawn npc after sensor established, or server will crash, out of time, maybe a bug
                
            self.first_call = False
        # Export LiDAR to cam0 transformation
        # tf_lidar_cam0 = gen.transform_lidar_to_camera(lidar_transform, left_transform)
        # with open(folder_output+"/lidar_to_cam0.txt", 'w') as posfile:
        #     posfile.write("#R(0,0) R(0,1) R(0,2) t(0) R(1,0) R(1,1) R(1,2) t(1) R(2,0) R(2,1) R(2,2) t(2)\n")
        #     posfile.write(str(tf_lidar_cam0[0][0])+" "+str(tf_lidar_cam0[0][1])+" "+str(tf_lidar_cam0[0][2])+" "+str(tf_lidar_cam0[0][3])+" ")
        #     posfile.write(str(tf_lidar_cam0[1][0])+" "+str(tf_lidar_cam0[1][1])+" "+str(tf_lidar_cam0[1][2])+" "+str(tf_lidar_cam0[1][3])+" ")
        #     posfile.write(str(tf_lidar_cam0[2][0])+" "+str(tf_lidar_cam0[2][1])+" "+str(tf_lidar_cam0[2][2])+" "+str(tf_lidar_cam0[2][3]))
        
        # # Export LiDAR to cam1 transformation
        # tf_lidar_cam1 = gen.transform_lidar_to_camera(lidar_transform, right_transform)
        # with open(folder_output+"/lidar_to_cam1.txt", 'w') as posfile:
        #     posfile.write("#R(0,0) R(0,1) R(0,2) t(0) R(1,0) R(1,1) R(1,2) t(1) R(2,0) R(2,1) R(2,2) t(2)\n")
        #     posfile.write(str(tf_lidar_cam1[0][0])+" "+str(tf_lidar_cam1[0][1])+" "+str(tf_lidar_cam1[0][2])+" "+str(tf_lidar_cam1[0][3])+" ")
        #     posfile.write(str(tf_lidar_cam1[1][0])+" "+str(tf_lidar_cam1[1][1])+" "+str(tf_lidar_cam1[1][2])+" "+str(tf_lidar_cam1[1][3])+" ")
        #     posfile.write(str(tf_lidar_cam1[2][0])+" "+str(tf_lidar_cam1[2][1])+" "+str(tf_lidar_cam1[2][2])+" "+str(tf_lidar_cam1[2][3]))


        # Launch MyCar and set MyCar to autopilot
        # MyCar.set_autopilot(True)

        # Pass to the next simulator frame to spawn sensors and to retrieve first data
        time.sleep(2)
        # self.world.tick()
        
        # VelodyneHDL64.init()
        gen.follow(MyCar.get_transform(), world)
        # self.counter += 1
    
    
        print("KITTI record beginning!")
        # All sensors produce first data at the same time (this ts)
        gen.Sensor.initial_ts = world.get_snapshot().timestamp.elapsed_seconds
        
        start_record = time.time()
        print("Start record : ")
        frame_current = 0

            
        if not self.first_call:
            initial_goal = self.goal
            while (frame_current < nbr_frame):
                frame_inspector = frame_current # get the value of last loop
                
                frame_current = self.VelodyneHDL64.save()
                self.normals.save() #Store location for Mycar
                
                # M- OP and IS not supported
                self.optical.save()
                self.RGB_left.save()
                self.RGB_right.save()
                self.ins_segmc.save()
                # depth.save()
                self.originDepth.save()
                self.SemSeg.save()
                self.IMU.save(MyCar,vehicles_list)#Here also stored all vehicles location
                if MyCar.is_at_traffic_light():
                    traffic_light = MyCar.get_traffic_light()
                    if traffic_light.get_state() == carla.TrafficLightState.Red:
                        traffic_light.set_state(carla.TrafficLightState.Green)

                # All sensors produce first data at the same time (this ts)
                gen.follow(MyCar.get_transform(), world)
                
                # M- debug
                
                if frame_current < 2:
                    print("Initializing!, frame = ", frame_current, self.world.get_settings().synchronous_mode)
                    # return None # to debug
                    
                else:
                    print("!Running loop for frame:", frame_current)
                    # self.world.tick() 
                
                
                # print("!Running loop for frame:", frame_current)
                
                if self.goal != initial_goal:
                    print("Goal changed! Exit the loop and re-route")
                    # self.on_goal
                    break
                
                if frame_current > frame_inspector:  # next frame
                    print("gooooooooooooooooooooo")
                    self.world.tick()
                
                
                time.sleep(0.1)
                # self.world.tick()    # Pass to the next simulator frame
                
        #VelodyneHDL64.save_poses()
        client.stop_recorder()
        print("Stop record")
        
        '''
        print('Destroying %d vehicles' % len(vehicles_list))
        client.apply_batch([carla.command.DestroyActor(x) for x in vehicles_list])
        vehicles_list.clear()
        
        # Stop walker controllers (list is [controller, actor, controller, actor ...])
        all_actors = world.get_actors(all_walkers_id)
        for i in range(0, len(all_walkers_id), 2):
            all_actors[i].stop()
        print('Destroying %d walkers' % (len(all_walkers_id)//2))
        client.apply_batch([carla.command.DestroyActor(x) for x in all_walkers_id])
        all_walkers_id.clear()
            
        print('Destroying MyCar')
        client.apply_batch([carla.command.DestroyActor(x) for x in actor_list])
        actor_list.clear()
            
        print("Elapsed time : ", time.time()-start_record)
        print()
        '''
        
        time.sleep(2.0)
    '''
    except Exception as e:
        print("An error occurred:", e)
        traceback.print_exc()

    finally:
        print("Elapsed total time : ", time.time()-start_record_full)
        world.apply_settings(init_settings)
        print('destroying actors')
        client.apply_batch([carla.command.DestroyActor(x) for x in actor_list])
        client.apply_batch([carla.command.DestroyActor(x) for x in vehicles_list])
        client.apply_batch([carla.command.DestroyActor(x) for x in all_walkers_id])
        print('done.!')
    '''
        
def main(args=None):
    """
    main function
    """
    roscomp.init('carla_waypoint_publisher', args)

    waypoint_converter = None
    try:
        waypoint_converter = CarlaToRosWaypointConverter()
        waypoint_converter.spin()
    except (RuntimeError, ROSException):
        pass
    except KeyboardInterrupt:
        roscomp.loginfo("User requested shut down.")
    except Exception as e:
        print("An error occurred:", e)
        traceback.print_exc()
    
    finally:
        roscomp.loginfo("Shutting down.")
        if waypoint_converter:
            waypoint_converter.destroy()
        roscomp.shutdown()




if __name__ == "__main__":
    main()
