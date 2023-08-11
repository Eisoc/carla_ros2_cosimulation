import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def launch_carla_spawn_object(context, *args, **kwargs):
    # workaround to use launch argument 'role_name' as a part of the string used for the spawn_point param name
    spawn_point_param_name = 'spawn_point_' + \
        launch.substitutions.LaunchConfiguration('role_name').perform(context)

    carla_spawn_objects_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(
                'carla_spawn_objects'), 'carla_example_ego_vehicle.launch.py')
        ),
        launch_arguments={
            'objects_definition_file': get_package_share_directory('carla_spawn_objects') + '/config/objects.json',
            spawn_point_param_name: launch.substitutions.LaunchConfiguration('spawn_point')
        }.items()
    )

    return [carla_spawn_objects_launch]

def launch_target_speed_publisher(context, *args, **kwargs):
    topic_name = "/carla/" + launch.substitutions.LaunchConfiguration('role_name').perform(context) + "/target_speed"
    data_string = "{'data': " + launch.substitutions.LaunchConfiguration('target_speed').perform(context) + "}"
    return [
        launch.actions.ExecuteProcess(
            output="screen",
            cmd=["ros2", "topic", "pub", topic_name,
                 "std_msgs/msg/Float64", data_string, "--qos-durability", "transient_local"],
            name='topic_pub_target_speed')]

def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='host',
            default_value='localhost'
        ),
        launch.actions.DeclareLaunchArgument(
            name='port',
            default_value='2000'
        ),
        launch.actions.DeclareLaunchArgument(
            name='town',
            default_value='Town01'
        ),
        launch.actions.DeclareLaunchArgument(
            name='timeout',
            default_value='10'
        ),
        launch.actions.DeclareLaunchArgument(
            name='synchronous_mode_wait_for_vehicle_control_command',
            default_value='False'
        ),
        launch.actions.DeclareLaunchArgument(
            name='fixed_delta_seconds',
            default_value='0.05'
        ),
        launch.actions.DeclareLaunchArgument(
            name='role_name',
            default_value='ego_vehicle'
        ),
        launch.actions.DeclareLaunchArgument(
            name='spawn_point',
            default_value='127.4,-195.4,2,0,0,180'
        ),
        launch.actions.DeclareLaunchArgument(
            name='target_speed',
            default_value='8.33' # in m/s
        ),
        launch.actions.DeclareLaunchArgument(
            name='avoid_risk',
            default_value='True'
        ),
        launch.actions.DeclareLaunchArgument(
            name='sigterm_timeout',def main():
    start_record_full = time.time()
    time_stop = 2.0
    nbr_frame = 3000 #MAX = 100000
    nbr_walkers = 0
    nbr_vehicles = 0

    actor_list = []
    vehicles_id_list = []
    all_walkers_id = []
    vehicles_list = []
    
    init_settings = None

    try:
        client = carla.Client('localhost', 2000)
        init_settings = carla.WorldSettings()
        client.set_timeout(100.0)
        print("Set town10 as our map")
        # print(client.get_available_maps())

        world = client.get_world()
        # world = client.load_world("Town10HD")
        # folder_output = "/media/stuf/data/Dataset_BrandNew%s/%s/generated" %(client.get_client_version(), world.get_map().name)
        folder_output = "//home/bing/Pictures/generator/Dataset_BrandNew%s/%s/generated" %(client.get_client_version(), world.get_map().name)
        os.makedirs(folder_output) if not os.path.exists(folder_output) else [os.remove(f) for f in glob.glob(folder_output+"/*") if os.path.isfile(f)]
        client.start_recorder(os.path.dirname(os.path.realpath(__file__))+"/"+folder_output+"/recording.log")
        
        # Weather
        world.set_weather(carla.WeatherParameters.ClearNoon)
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
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.01
        settings.no_rendering_mode = False
        world.apply_settings(settings)

        # Create our vehicle
        blueprint_library = world.get_blueprint_library()
        bp_MyCar = blueprint_library.find('vehicle.lincoln.mkz_2020')
        bp_MyCar.set_attribute('color', '228, 239, 241')
        bp_MyCar.set_attribute('role_name', 'MyCar')

        #Our vehicle's start pose
        # start_pose = random.choice(world.get_map().get_spawn_points())#
        
        # set goal
        # end_point = random.choice(world.get_map().get_spawn_points())
        # road_option = carla.RoadOption.LANEFOLLOW  # 设定车辆跟随道路行驶
        # route = world.get_map().calculate_shortest_route(start_pose.location, end_point.location, road_option)
        '''
        start_pose_str = '127.4, -195.4, 2, 0, 0, 180'
        start_pose_values = [float(x) for x in start_pose_str.split(',')]

        location = carla.Location(x=start_pose_values[0], y=start_pose_values[1], z=start_pose_values[2])
        rotation = carla.Rotation(pitch=start_pose_values[3], yaw=start_pose_values[4], roll=start_pose_values[5])
        start_pose = carla.Transform(location, rotation)
        '''
        
        MyCar = world.spawn_actor(bp_MyCar, spawn_point)
        actor_list.append(MyCar)
        print('Created %s' % MyCar)

        # Spawn vehicles and walkers
        vehicles_id_list = gen.spawn_npc(client, nbr_vehicles, nbr_walkers, vehicles_id_list, all_walkers_id)
        print(vehicles_id_list)
        for x in vehicles_id_list:
            vehicles_list.append(world.get_actor(x))

        #Set instance id
        # world.set_instance_tagging_style("actor_id")

        # Wait for MyCar to stop
        start = world.get_snapshot().timestamp.elapsed_seconds
        print("Waiting for MyCar to stop ...")
        while world.get_snapshot().timestamp.elapsed_seconds-start < time_stop: world.tick()
        print("MyCar stopped")

        # Set sensors transformation from MyCar
        lidar_transform     = carla.Transform(carla.Location(x=0, y=0, z=1.80))
        left_transform = carla.Transform(carla.Location(x=2.0, y=-0.2, z=1.4))
        right_transform = carla.Transform(carla.Location(x=2.0, y=0.2, z=1.4))
        with open(folder_output+"/start_pose.txt", 'a') as pose :
            print("left camera pose: " + str(left_transform)+ "right camera pose: " + str(right_transform),file=pose)

        # Take a screenshot#We dont need this
        #gen.screenshot(MyCar, world, actor_list, folder_output, carla.Transform(carla.Location(x=0.0, y=0, z=2.0), carla.Rotation(pitch=0, yaw=0, roll=0)))

        # Create our sensors
        gen.RGB_left.sensor_id_glob = 0
        gen.RGB_right.sensor_id_glob = 0
        gen.IS.sensor_id_glob = 10
        gen.Depth.sensor_id_glob = 20
        gen.HDL64E.sensor_id_glob = 100
        gen.IMU.sensor_id_glob = 0
        gen.Optical.sensor_id_glob = 0

        VelodyneHDL64 = gen.HDL64E(MyCar, world, actor_list, folder_output, lidar_transform)
        RGB_left = gen.RGB_left(MyCar, world, actor_list, folder_output, left_transform)
        RGB_right = gen.RGB_right(MyCar, world, actor_list, folder_output, right_transform)
        ins_segmc = gen.IS(MyCar, world, actor_list, folder_output, right_transform)
        # depth = gen.Depth(MyCar, world, actor_list, folder_output, right_transform)
        originDepth = gen.Original_Depth(MyCar, world, actor_list, folder_output, right_transform)
        normals = gen.Normal(MyCar, world, actor_list, folder_output, right_transform)
        optical = gen.Optical(MyCar, world, actor_list, folder_output, right_transform)
        IMU = gen.IMU(MyCar, world, actor_list, folder_output, right_transform)
        SemSeg = gen.SS(MyCar, world, actor_list, folder_output, right_transform)
    


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
        world.tick()
        
        # VelodyneHDL64.init()
        gen.follow(MyCar.get_transform(), world)
        

        
      
        # All sensors produce first data at the same time (this ts)
        gen.Sensor.initial_ts = world.get_snapshot().timestamp.elapsed_seconds
        
        start_record = time.time()
        print("Start record : ")
        frame_current = 0
        while (frame_current < nbr_frame):
            frame_current = VelodyneHDL64.save()
            normals.save() #Store location for Mycar
            optical.save()
            RGB_left.save()
            RGB_right.save()
            ins_segmc.save()
            # depth.save()
            originDepth.save()
            SemSeg.save()
            IMU.save(MyCar,vehicles_list)#Here also stored all vehicles location
            if MyCar.is_at_traffic_light():
                traffic_light = MyCar.get_traffic_light()
                if traffic_light.get_state() == carla.TrafficLightState.Red:
                    
                    traffic_light.set_state(carla.TrafficLightState.Green)
            # All sensors produce first data at the same time (this ts)
            gen.follow(MyCar.get_transform(), world)
            world.tick()    # Pass to the next simulator frame
        
        #VelodyneHDL64.save_poses()
        client.stop_recorder()
        print("Stop record")
        
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
            
        time.sleep(2.0)

    finally:
        print("Elapsed total time : ", time.time()-start_record_full)
        world.apply_settings(init_settings)
        print('destroying actors')
        client.apply_batch([carla.command.DestroyActor(x) for x in actor_list])
        client.apply_batch([carla.command.DestroyActor(x) for x in vehicles_list])
        client.apply_batch([carla.command.DestroyActor(x) for x in all_walkers_id])
        print('done.')
        

            default_value='15'
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'carla_ros_bridge'), 'carla_ros_bridge.launch.py')
            ),
            launch_arguments={
                'host': launch.substitutions.LaunchConfiguration('host'),
                'port': launch.substitutions.LaunchConfiguration('port'),
                'town': launch.substitutions.LaunchConfiguration('town'),
                'timeout': launch.substitutions.LaunchConfiguration('timeout'),
                'synchronous_mode_wait_for_vehicle_control_command': launch.substitutions.LaunchConfiguration('synchronous_mode_wait_for_vehicle_control_command'),
                'fixed_delta_seconds': launch.substitutions.LaunchConfiguration('fixed_delta_seconds')
            }.items()
        ),
        launch.actions.OpaqueFunction(function=launch_carla_spawn_object),
        launch.actions.OpaqueFunction(function=launch_target_speed_publisher),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'carla_ad_agent'), 'carla_ad_agent.launch.py')
            ),
            launch_arguments={
                'role_name': launch.substitutions.LaunchConfiguration('role_name'),
                'avoid_risk': launch.substitutions.LaunchConfiguration('avoid_risk')
            }.items()
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'carla_waypoint_publisher'), 'carla_waypoint_publisher.launch.py')
            ),
            launch_arguments={
                'host': launch.substitutions.LaunchConfiguration('host'),
                'port': launch.substitutions.LaunchConfiguration('port'),
                'timeout': launch.substitutions.LaunchConfiguration('timeout'),
                'role_name': launch.substitutions.LaunchConfiguration('role_name')
            }.items()
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'carla_manual_control'), 'carla_manual_control.launch.py')
            ),
            launch_arguments={
                'role_name': launch.substitutions.LaunchConfiguration('role_name')
            }.items()
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
