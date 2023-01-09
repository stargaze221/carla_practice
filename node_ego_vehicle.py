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

from tools.constants import COLOR_BLACK, COLOR_WHITE, PIXELS_PER_METER, PIXELS_AHEAD_VEHICLE
from tools.constants import COLOR_ALUMINIUM_4, COLOR_PLUM_0, COLOR_SKY_BLUE_0, COLOR_CHOCOLATE_1, COLOR_CHAMELEON_0, COLOR_ALUMINIUM_1



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

        scaled_original_size = self.original_surface_size * (1.5)
        self.hero_surface = pygame.Surface((scaled_original_size, scaled_original_size)).convert()
        self.result_surface = pygame.Surface((self.surface_size, self.surface_size)).convert()
        self.result_surface.set_colorkey(COLOR_BLACK)

        # Render Actors
        self.actors_surface = pygame.Surface((self.map_image.surface.get_width(), self.map_image.surface.get_height()))
        self.actors_surface.set_colorkey(COLOR_BLACK)

        self.border_round_surface = pygame.Surface((self.original_surface_size,self.original_surface_size), pygame.SRCALPHA).convert()
        self.border_round_surface.set_colorkey(COLOR_WHITE)
        self.border_round_surface.fill(COLOR_BLACK)

        # Used for Hero Mode, draws the map contained in a circle with white border
        center_offset = (int(self.original_surface_size / 2), int(self.original_surface_size/ 2))



        # Apply clipping rect
        clipping_rect = pygame.Rect(0,
                                    0,
                                    300,
                                    300)
        pygame.draw.rect(self.border_round_surface, COLOR_ALUMINIUM_1, clipping_rect)
        pygame.draw.rect(self.border_round_surface, COLOR_WHITE, clipping_rect)
        
        
        
        # pygame.draw.circle(self.border_round_surface, COLOR_ALUMINIUM_1, center_offset, int(self.original_surface_size / 2))
        # pygame.draw.circle(self.border_round_surface, COLOR_WHITE, center_offset, int((self.original_surface_size - 8) / 2))


    def observe(self):


        actors = self.world.get_actors()

        # We store the transforms also so that we avoid having transforms of
        # previous tick and current tick when rendering them.
        self.actors_with_transforms = [(actor, actor.get_transform()) for actor in actors]
        if self.hero_actor is not None:
            self.hero_transform = self.hero_actor.get_transform()

        return 0

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
            color = COLOR_SKY_BLUE_0
            if int(v[0].attributes['number_of_wheels']) == 2:
                color = COLOR_CHOCOLATE_1
            if v[0].attributes['role_name'] == 'hero':
                color = COLOR_CHAMELEON_0
            # Compute bounding box points
            bb = v[0].bounding_box.extent
            # corners = [carla.Location(x=-bb.x, y=-bb.y),
            #            carla.Location(x=bb.x - 0.8, y=-bb.y),
            #            carla.Location(x=bb.x, y=0),
            #            carla.Location(x=bb.x - 0.8, y=bb.y),
            #            carla.Location(x=-bb.x, y=bb.y),
            #            carla.Location(x=-bb.x, y=-bb.y)
            #            ]
            # v[1].transform(corners)
            # corners = [world_to_pixel(p) for p in corners]
            # pygame.draw.lines(surface, color, False, corners, int(math.ceil(4.0 * self.map_image.scale)))

            corners = [
                carla.Location(x=-bb.x, y=-bb.y),
                carla.Location(x=bb.x, y=-bb.y),
                carla.Location(x=bb.x, y=bb.y),
                carla.Location(x=-bb.x, y=bb.y)]
            v[1].transform(corners)
            corners = [world_to_pixel(p) for p in corners]
            pygame.draw.polygon(surface, color, corners)


            
    def render_actors(self, surface, vehicles, traffic_lights, speed_limits, walkers):
        """Renders all the actors"""
        # Static actors
        # self._render_traffic_lights(surface, [tl[0] for tl in traffic_lights], self.map_image.world_to_pixel)
        # self._render_speed_limits(surface, [sl[0] for sl in speed_limits], self.map_image.world_to_pixel,
        #                           self.map_image.world_to_pixel_width)

        # Dynamic actors
        self._render_vehicles(surface, vehicles, self.map_image.world_to_pixel)
        self._render_walkers(surface, walkers, self.map_image.world_to_pixel)


    def render(self):

        self.observe()

        self.result_surface.fill(COLOR_BLACK)

        # Split the actors by vehicle type id
        vehicles, traffic_lights, speed_limits, walkers = self._split_actors()


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

            self.border_round_surface.set_clip(clipping_rect)


            self.hero_surface.fill(COLOR_ALUMINIUM_4)
            self.hero_surface.blit(self.result_surface, (-translation_offset[0],
                                                         -translation_offset[1]))

            rotated_result_surface = pygame.transform.rotozoom(self.hero_surface, angle, 1).convert()

            display_size = self.display_man.get_display_size()

            center = (display_size[0] / 2, display_size[1] / 2)
            rotation_pivot = rotated_result_surface.get_rect(center=center)
            offset = self.display_man.get_display_offset(self.display_pos)


            ### <--- Temp for clipping 


            # clipping_rect = pygame.Rect(0,
            #                             0,
            #                             self.hero_surface.get_width(),
            #                             self.hero_surface.get_height())

            # rotated_result_surface.set_clip(clipping_rect)

            ### <--- Temp for clipping


            self.display_man.display.blit(rotated_result_surface, (offset[0]+rotation_pivot[0], offset[1]+rotation_pivot[1]))

            self.display_man.display.blit(self.border_round_surface, (offset[0], offset[1]))




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
        bp.set_attribute('role_name', 'hero')
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


        # Map Manager
        MapManager(world, display_manager, vehicle, display_pos=[0,3])


        SensorManager(world, display_manager, 'None', None, None, None, display_pos=[1, 0])  # To add black area.
        SensorManager(world, display_manager, 'None', None, None, None, display_pos=[1, 2])  # To add black area.
                      

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
                      vehicle, {'channels' : '64', 'range' : '100',  'points_per_second': '250000', 'rotation_frequency': '20'}, display_pos=[1, 3])
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