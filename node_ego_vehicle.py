#!/usr/bin/env python3

import glob
import os
import sys
import math
import argparse
import random
import time
import carla
import numpy as np


try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

try:
    import pygame
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_q
    from pygame.locals import K_p
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

from tools.settings import LOOP_FREQ
from tools.sensors import SensorManager
from tools.display import DisplayManager
from tools.map import MapImage, Util
from tools.constants import COLOR_BLACK, COLOR_WHITE, PIXELS_PER_METER, PIXELS_AHEAD_VEHICLE
from tools.constants import COLOR_ALUMINIUM_4, COLOR_PLUM_0, COLOR_SKY_BLUE_0, COLOR_CHOCOLATE_1, COLOR_CHAMELEON_0, COLOR_ALUMINIUM_1



class MapManager(object):

    def __init__(self, world, mapimage, display_manager, vehicle, display_pos=[1, 0]):

        self.world = world
        self.map_image = mapimage

        # Add the sensor, map manager, to the dislay manager.
        self.display_pos = display_pos
        self.display_man = display_manager
        self.display_man.add_sensor(self)

        # Get the ego vehicle
        self.hero_actor = vehicle

        # Pygame surface to be used within the display manager. 
        self.original_surface_size = min(self.display_man.get_display_size())
        self.surface_size = self.map_image.big_map_surface.get_width()
        self.scaled_size = int(self.surface_size)
        scaled_original_size = self.original_surface_size * (1.5)  # Need to enlarge it due to the rotation.
        self.hero_surface = pygame.Surface((scaled_original_size, scaled_original_size)).convert()
        self.result_surface = pygame.Surface((self.surface_size, self.surface_size)).convert() # The result surface's size is set equal to the big map.
        self.result_surface.set_colorkey(COLOR_BLACK) # "set_colorkey" choose transparent color.

        # Pygame surface to draw actors, e.g., vehicles and pedestrians.
        self.actors_surface = pygame.Surface((self.map_image.surface.get_width(), self.map_image.surface.get_height()))
        self.actors_surface.set_colorkey(COLOR_BLACK) 

        # Pygame surface to clipping the big map.
        self.border_surface = pygame.Surface((self.original_surface_size,self.original_surface_size), pygame.SRCALPHA).convert()
        self.border_surface.set_colorkey(COLOR_WHITE)
        self.border_surface.fill(COLOR_BLACK)

        clipping_rect = pygame.Rect(0, 0, self.original_surface_size, self.original_surface_size)
        pygame.draw.rect(self.border_surface, COLOR_ALUMINIUM_1, clipping_rect)
        pygame.draw.rect(self.border_surface, COLOR_WHITE, clipping_rect)
        

    def render(self):

        vehicles, traffic_lights, speed_limits, walkers = self.observe()

        # Zoom in and out
        scale_factor = 0.3
        self.scaled_size = int(self.map_image.width * scale_factor)
        self.map_image.scale_map(scale_factor)

        # Render Actors
        self.actors_surface.fill(COLOR_BLACK)
        self.render_actors(
            self.actors_surface,
            vehicles,
            traffic_lights,
            speed_limits,
            walkers)

        # Blit surfaces
        surfaces = ((self.map_image.surface, (0, 0)),
                    (self.actors_surface, (0, 0))
                    )
        Util.blits(self.result_surface, surfaces)
        
        # Clip the result surface using the location of the ego vehicle.
        hero_location_screen = self.map_image.world_to_pixel(self.hero_transform.location)
        hero_front = self.hero_transform.get_forward_vector()
        translation_offset = (hero_location_screen[0] - self.hero_surface.get_width() / 2 + hero_front.x * PIXELS_AHEAD_VEHICLE,
                                (hero_location_screen[1] - self.hero_surface.get_height() / 2 + hero_front.y * PIXELS_AHEAD_VEHICLE))
        
        clipping_rect = pygame.Rect(translation_offset[0],
                                    translation_offset[1],
                                    self.hero_surface.get_width(),
                                    self.hero_surface.get_height())
        self.result_surface.set_clip(clipping_rect)
        


        self.hero_surface.fill(COLOR_ALUMINIUM_4)
        self.hero_surface.blit(self.result_surface, (-translation_offset[0],-translation_offset[1]))

        angle = 0.0 if self.hero_actor is None else self.hero_transform.rotation.yaw + 90.0
        rotated_result_surface = pygame.transform.rotozoom(self.hero_surface, angle, 1).convert()

        display_size = self.display_man.get_display_size()

        center = (display_size[0] / 2, display_size[1] / 2)
        rotation_pivot = rotated_result_surface.get_rect(center=center)
        offset = self.display_man.get_display_offset(self.display_pos)

        self.display_man.display.blit(rotated_result_surface, (offset[0]+rotation_pivot[0], offset[1]+rotation_pivot[1]))
        self.display_man.display.blit(self.border_surface, (offset[0], offset[1]))


    def observe(self):
        actors = self.world.get_actors()
        # We store the transforms also so that we avoid having transforms of
        # previous tick and current tick when rendering them.
        self.actors_with_transforms = [(actor, actor.get_transform()) for actor in actors]
        if self.hero_actor is not None:
            self.hero_transform = self.hero_actor.get_transform()

        return self._split_actors()

    def render_actors(self, surface, vehicles, traffic_lights, speed_limits, walkers):
        """Renders all the actors"""

        # Dynamic actors
        self._render_vehicles(surface, vehicles, self.map_image.world_to_pixel)
        self._render_walkers(surface, walkers, self.map_image.world_to_pixel)


    def _split_actors(self):
        """Splits the retrieved actors by type id"""
        vehicles = []
        traffic_lights = []
        speed_limits = []
        walkers = []

        for actor_with_transform in self.actors_with_transforms:
            actor = actor_with_transform[0]
            if 'vehicle' in actor.type_id:
                vehicles.append(actor_with_transform)
            elif 'traffic_light' in actor.type_id:
                traffic_lights.append(actor_with_transform)
            elif 'speed_limit' in actor.type_id:
                speed_limits.append(actor_with_transform)
            elif 'walker.pedestrian' in actor.type_id:
                walkers.append(actor_with_transform)

        return (vehicles, traffic_lights, speed_limits, walkers)


    def _render_walkers(self, surface, list_w, world_to_pixel):
        """Renders the walkers' bounding boxes"""
        for w in list_w:
            color = COLOR_PLUM_0
            # Compute bounding box points
            bb = w[0].bounding_box.extent
            corners = [
                carla.Location(x=-bb.x, y=-bb.y),
                carla.Location(x=bb.x, y=-bb.y),
                carla.Location(x=bb.x, y=bb.y),
                carla.Location(x=-bb.x, y=bb.y)]

            w[1].transform(corners)
            corners = [world_to_pixel(p) for p in corners]
            pygame.draw.polygon(surface, color, corners)

    def _render_vehicles(self, surface, list_v, world_to_pixel):
        """Renders the vehicles' bounding boxes"""
        for v in list_v:

            distance = v[1].location.distance(self.hero_actor.get_transform().location)

            if distance < 50: # Filter for distance from ego vehicle
                color = COLOR_SKY_BLUE_0
                if int(v[0].attributes['number_of_wheels']) == 2:
                    color = COLOR_CHOCOLATE_1
                if v[0].attributes['role_name'] == 'hero':
                    color = COLOR_CHAMELEON_0
                # Compute bounding box points
                bb = v[0].bounding_box.extent

                corners = [
                    carla.Location(x=-bb.x, y=-bb.y),
                    carla.Location(x=bb.x, y=-bb.y),
                    carla.Location(x=bb.x, y=bb.y),
                    carla.Location(x=-bb.x, y=bb.y)]
                v[1].transform(corners)
                corners = [world_to_pixel(p) for p in corners]
                pygame.draw.polygon(surface, color, corners)



def run_carla_node(args, client):

    pygame.init()
    pygame.font.init()
    pygame.joystick.init()

    # Display Manager organize all the sensors an its display in a window
    # If can easily configure the grid and the total window size
    display_manager = DisplayManager(grid_size=[3, 4], window_size=[args.width, args.height])

    # Initialize joystick
    try:
        joystick = pygame.joystick.Joystick(0)
        joystick.init()
        print(joystick.get_name())
        print(joystick.get_numaxes())
        
    except:
        print('no...')               

    vehicle = None
    vehicle_list = []

    try:        
        # Getting the world and
        world = client.get_world()
        town_map = world.get_map()
        original_settings = world.get_settings()
        settings = world.get_settings()
        settings.no_rendering_mode = True
        world.apply_settings(settings)
        print(settings)

        # Initialize the static map image using MapImage.
        mapimage = MapImage(world, town_map, PIXELS_PER_METER, False, False, False)

        # Instanciating the vehicle to which we attached the sensors
        bp = world.get_blueprint_library().filter('charger_2020')[0]
        bp.set_attribute('role_name', 'hero')
        vehicle = world.spawn_actor(bp, random.choice(world.get_map().get_spawn_points()))
        print('Ego vehicle ID:', vehicle.id)
        vehicle_list.append(vehicle)
        vehicle.set_autopilot(False)

        # Initialize the vehicle control
        vehicle_control = vehicle.get_control()
        vehicle_control.throttle = 0
        vehicle_control.gear = 1
        vehicle_control.brake = 0
        vehicle_control.hand_brake = False
        vehicle_control.manual_gear_shift = False
        vehicle_control.reverse = False
        vehicle_control.steer = 0

        # Map Manager
        MapManager(world, mapimage, display_manager, vehicle, display_pos=[0,3])

        # Blank displays
        SensorManager(world, display_manager, 'None', None, None, None, display_pos=[1, 0])  # To add black area.
        #SensorManager(world, display_manager, 'None', None, None, None, display_pos=[1, 2])  # To add black area.
                      
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
        SensorManager(world, display_manager, 'LiDAR', carla.Transform(carla.Location(x=0, z=2.4), carla.Rotation(yaw=+90)), 
                      vehicle, {'channels' : '64', 'range' : '100',  'points_per_second': '250000', 'rotation_frequency': '20'}, display_pos=[1, 2])

        SensorManager(world, display_manager, 'SemanticSegmenation', carla.Transform(carla.Location(x=0, z=40), carla.Rotation(pitch=-90)), 
                      vehicle, {}, display_pos=[1, 3])

        SensorManager(world, display_manager, 'SemanticSegmenation', carla.Transform(carla.Location(x=0, z=2.4), carla.Rotation(yaw=-90)), 
                      vehicle, {}, display_pos=[2, 0])
        SensorManager(world, display_manager, 'SemanticSegmenation', carla.Transform(carla.Location(x=0, z=2.4), carla.Rotation(yaw=+00)), 
                      vehicle, {}, display_pos=[2, 1])
        SensorManager(world, display_manager, 'SemanticSegmenation', carla.Transform(carla.Location(x=0, z=2.4), carla.Rotation(yaw=+90)), 
                      vehicle, {}, display_pos=[2, 2])
        SensorManager(world, display_manager, 'SemanticSegmenation', carla.Transform(carla.Location(x=0, z=2.4), carla.Rotation(yaw=180)), 
                      vehicle, {}, display_pos=[2, 3])

        #Simulation loop
        call_exit = False
        clock = pygame.time.Clock()

        start_tick = pygame.time.get_ticks()
        if_auto_pilot = False

        while True:
            # Carla Tick
            clock.tick(LOOP_FREQ)
            world.wait_for_tick() # asynchronous mode

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
                print(fps, 'error in loop frequency', int(passed_ticks/1000))
            
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
        help='window resolution (default: 1200x900)')

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