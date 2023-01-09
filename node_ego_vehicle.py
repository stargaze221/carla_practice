#!/usr/bin/env python3

import glob
import os
import sys
import math

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
import argparse
import random
import time
import numpy as np

try:
    import pygame
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_q
    from pygame.locals import K_p
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

from tools.settings import LOOP_FREQ
from tools.sensors import SensorManager, DisplayManager
from tools.map import MapImage, Util
from tools.constants import COLOR_BLACK, COLOR_WHITE, PIXELS_PER_METER, PIXELS_AHEAD_VEHICLE, COLOR_ALUMINIUM_4

#<--------- Here, let me work on it as a class




class MapManager(object):

    def __init__(self, world, display_manager, vehicle, display_pos=[1, 0]):

        self.world = world
        self.town_map = world.get_map()
        self.display_pos = display_pos
        self.display_man = display_manager
        self.display_man.add_sensor(self)
        self.hero_actor = vehicle

        self.map_image = MapImage(world, self.town_map, PIXELS_PER_METER, False, False, False)

        # I need to plot it in the surface below. At this moment 
        self.original_surface_size = min(self.display_man.get_display_size())
        self.surface_size = self.map_image.big_map_surface.get_width()
        self.scaled_size = int(self.surface_size)

        scaled_original_size = self.original_surface_size * (1.0 / 0.9)
        self.hero_surface = pygame.Surface((scaled_original_size, scaled_original_size)).convert()
        self.result_surface = pygame.Surface((self.surface_size, self.surface_size)).convert()
        self.result_surface.set_colorkey(COLOR_BLACK)

    def observe(self):

        # transform = self.ego_vehicle.get_transform()
        # print(transform)

        actors = self.world.get_actors()

        # We store the transforms also so that we avoid having transforms of
        # previous tick and current tick when rendering them.
        self.actors_with_transforms = [(actor, actor.get_transform()) for actor in actors]
        if self.hero_actor is not None:
            self.hero_transform = self.hero_actor.get_transform()

        return 0


    def render(self):

        self.observe()

        self.result_surface.fill(COLOR_BLACK)

        # Zoom in and out
        scale_factor = 0.5
        self.scaled_size = int(self.map_image.width * scale_factor)
        self.map_image.scale_map(scale_factor)

        # Blit surfaces
        surfaces = ((self.map_image.surface, (0, 0)),
                    (self.map_image.surface, (0, 0))
                    )

        angle = 0.0 if self.hero_actor is None else self.hero_transform.rotation.yaw + 90.0

        if self.hero_actor is not None:
            # Hero Mode
            hero_location_screen = self.map_image.world_to_pixel(self.hero_transform.location)
            hero_front = self.hero_transform.get_forward_vector()
            translation_offset = (hero_location_screen[0] - self.hero_surface.get_width() / 2 + hero_front.x * PIXELS_AHEAD_VEHICLE,
                                  (hero_location_screen[1] - self.hero_surface.get_height() / 2 + hero_front.y * PIXELS_AHEAD_VEHICLE))

            # Apply clipping rect
            clipping_rect = pygame.Rect(translation_offset[0],
                                        translation_offset[1],
                                        self.hero_surface.get_width(),
                                        self.hero_surface.get_height())


            self.result_surface.set_clip(clipping_rect)

            Util.blits(self.result_surface, surfaces)


            self.hero_surface.fill(COLOR_ALUMINIUM_4)
            self.hero_surface.blit(self.result_surface, (-translation_offset[0],
                                                         -translation_offset[1]))

            rotated_result_surface = pygame.transform.rotozoom(self.hero_surface, angle, 0.9).convert()

            display_size = self.display_man.get_display_size()

            center = (display_size[0] / 2, display_size[1] / 2)
            rotation_pivot = rotated_result_surface.get_rect(center=center)
            offset = self.display_man.get_display_offset(self.display_pos)
            


            self.display_man.display.blit(rotated_result_surface, (offset[0]+rotation_pivot[0], offset[1]+rotation_pivot[1]))




        #self.map_image.big_map_surface.set_clip(clipping_rect)

        #offset = self.display_man.get_display_offset(self.display_pos)
        #self.display_man.display.blit(self.map_image.big_map_surface, offset)





#<----------------




def run_carla_node(args, client):

    pygame.init()
    pygame.font.init()
    pygame.joystick.init()

    # Initialize joystick
    try:
        joystick = pygame.joystick.Joystick(0)
        joystick.init()
        print(joystick.get_name())
        print(joystick.get_numaxes())
        
    except:
        print('no...')               
    

    display_manager = None
    vehicle = None
    vehicle_list = []

    # Initialize the joysticks.

    try:
        # Getting the world and
        world = client.get_world()
        original_settings = world.get_settings()

        print(original_settings)

        # Instanciating the vehicle to which we attached the sensors
        bp = world.get_blueprint_library().filter('charger_2020')[0]
        vehicle = world.spawn_actor(bp, random.choice(world.get_map().get_spawn_points()))

        print(vehicle.id)

        vehicle_list.append(vehicle)
        vehicle.set_autopilot(False)

        vehicle_control = vehicle.get_control()
        vehicle_control.throttle = 0
        vehicle_control.gear = 1
        vehicle_control.brake = 0
        vehicle_control.hand_brake = False
        vehicle_control.manual_gear_shift = False
        vehicle_control.reverse = False
        vehicle_control.steer = 0
        

        # Display Manager organize all the sensors an its display in a window
        # If can easily configure the grid and the total window size
        display_manager = DisplayManager(grid_size=[3, 4], window_size=[args.width, args.height])

        # Then, SensorManager can be used to spawn RGBCamera, LiDARs and SemanticLiDARs as needed
        # and assign each of them to a grid position, 
        SensorManager(world, display_manager, 'RGBCamera', carla.Transform(carla.Location(x=0, z=2.4), carla.Rotation(yaw=-90)), 
                      vehicle, {}, display_pos=[0, 0])
        SensorManager(world, display_manager, 'RGBCamera', carla.Transform(carla.Location(x=0, z=2.4), carla.Rotation(yaw=+00)), 
                      vehicle, {}, display_pos=[0, 1])
        SensorManager(world, display_manager, 'RGBCamera', carla.Transform(carla.Location(x=0, z=2.4), carla.Rotation(yaw=+90)), 
                      vehicle, {}, display_pos=[0, 2])
        SensorManager(world, display_manager, 'RGBCamera', carla.Transform(carla.Location(x=0, z=2.4), carla.Rotation(yaw=180)), 
                      vehicle, {}, display_pos=[1, 1])

        SensorManager(world, display_manager, 'LiDAR', carla.Transform(carla.Location(x=0, z=2.4), carla.Rotation(yaw=+00)), 
                      vehicle, {'channels' : '64', 'range' : '100',  'points_per_second': '250000', 'rotation_frequency': '20'}, display_pos=[0, 3])
        SensorManager(world, display_manager, 'SemanticLiDAR', carla.Transform(carla.Location(x=0, z=2.4), carla.Rotation(yaw=+00)), 
                      vehicle, {'channels' : '64', 'range' : '100', 'points_per_second': '100000', 'rotation_frequency': '20'}, display_pos=[1, 3])

        SensorManager(world, display_manager, 'SemanticSegmenation', carla.Transform(carla.Location(x=0, z=2.4), carla.Rotation(yaw=-90)), 
                      vehicle, {}, display_pos=[2, 0])
        SensorManager(world, display_manager, 'SemanticSegmenation', carla.Transform(carla.Location(x=0, z=2.4), carla.Rotation(yaw=+00)), 
                      vehicle, {}, display_pos=[2, 1])
        SensorManager(world, display_manager, 'SemanticSegmenation', carla.Transform(carla.Location(x=0, z=2.4), carla.Rotation(yaw=+90)), 
                      vehicle, {}, display_pos=[2, 2])
        SensorManager(world, display_manager, 'SemanticSegmenation', carla.Transform(carla.Location(x=0, z=2.4), carla.Rotation(yaw=180)), 
                      vehicle, {}, display_pos=[2, 3])

        # Map Manager
        mapmanager = MapManager(world, display_manager, vehicle, display_pos=[1,0])



        

        #Simulation loop
        call_exit = False
        clock = pygame.time.Clock()

        start_tick = pygame.time.get_ticks()
        if_auto_pilot = False

        while True:
            # Carla Tick
            clock.tick(LOOP_FREQ)
            world.wait_for_tick()


            # Render received data
            display_manager.render()

            # Get Joystick Input and Apply to Carla #
            try:
                vehicle_control.throttle = (joystick.get_axis(2) + 1)/2
                vehicle_control.brake = (joystick.get_axis(5) + 1)/2
                vehicle_control.steer = joystick.get_axis(3)
            except:
                vehicle_control.throttle = 0
                vehicle_control.brake = 0
                vehicle_control.steer = 0
            vehicle.apply_control(vehicle_control)

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    call_exit = True
                elif event.type == pygame.JOYBUTTONDOWN:
                    if joystick.get_button(4):
                        vehicle_control.reverse ^= True
                        print("Reverse Gear", vehicle_control.reverse)
                elif event.type == pygame.KEYDOWN:
                    if event.key == K_ESCAPE or event.key == K_q:
                        call_exit = True
                    elif event.key == K_p:
                        if_auto_pilot ^= True
                        vehicle.set_autopilot(if_auto_pilot)


            if call_exit:
                break

            

            # Sleep for Set Rate
            passed_ticks = pygame.time.get_ticks() - start_tick
            fps = clock.get_fps()
            if (passed_ticks > 5*1000) and abs(fps - LOOP_FREQ) > 0.1:
                print(fps, 'error in loop frequency')
            
    finally:
        if display_manager:
            display_manager.destroy()

        client.apply_batch([carla.command.DestroyActor(x) for x in vehicle_list])

        world.apply_settings(original_settings)

def main():
    argparser = argparse.ArgumentParser(
        description='CARLA Sensor tutorial')
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '--res',
        metavar='WIDTHxHEIGHT',
        default='1200x900',
        help='window resolution (default: 1280x720)')

    args = argparser.parse_args()

    args.width, args.height = [int(x) for x in args.res.split('x')]

    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(5.0)

        run_carla_node(args, client)

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')


if __name__ == '__main__':

    main()