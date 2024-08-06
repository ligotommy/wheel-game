"""
Tools for editing/using tilemaps in pygame.
"""

import math
import pygame
import json


class Tilemap:
    """
    Tilemap(mapfile="file_path.json",
        tile_files={"tile_name": "tile_pic_path.png"},
        surface=(the pygame surface),
        grid_unit=(the size of each tile),
        zoom_multiplier=1(by default)
    )

    Provides tools to edit/use tilemaps using pygame.
    """

    def __init__(self, mapfile: str, tile_files: dict[str, dict], surface: pygame.Surface,
                 grid_unit: int, zoom_multiplier: float = 1):
        self.GRID_UNIT = grid_unit
        self._zoom_multiplier = zoom_multiplier
        self.surface = surface
        self.screen_width = self.surface.get_width()
        self.screen_height = self.surface.get_height()
        self.mapfile = mapfile
        self.tile_files = tile_files
        self.tilemap = self.load_map()

    @property
    def zoom_mul(self):
        return self._zoom_multiplier

    @zoom_mul.setter
    def zoom_mul(self, new_zoom):
        if new_zoom > 0:
            self._zoom_multiplier = new_zoom
        else:
            raise Exception("Zoom multiplier must be positive.")

    def world_to_screen(self, x, y, camera):
        x -= camera["x"]
        y -= camera["y"]
        return [round(x * self.GRID_UNIT * self._zoom_multiplier), round(y * -self.GRID_UNIT * self._zoom_multiplier)]

    def world_to_screen_diff(self, x, y):
        return [x * self.GRID_UNIT * self._zoom_multiplier, y * -self.GRID_UNIT * self._zoom_multiplier]

    def screen_to_world(self, x, y, camera):
        x /= (self.GRID_UNIT * self._zoom_multiplier)
        y /= (-self.GRID_UNIT * self._zoom_multiplier)
        return [x+camera["x"], y+camera["y"]]

    def screen_to_world_diff(self, x, y):
        return [x / (self.GRID_UNIT * self._zoom_multiplier), y / (-self.GRID_UNIT * self._zoom_multiplier)]

    def load_map(self) -> dict[str, dict[str, list[list[int]] | int | float]]:
        with open(self.mapfile, "r") as tiles:
            data = json.load(tiles)
            return data

    def draw_map(self, camera):
        for tile_name, properties_dict in self.tilemap.items():
            img = pygame.image.load(self.tile_files[tile_name]["path"])
            image = pygame.transform.scale(img, (self.GRID_UNIT*self._zoom_multiplier,
                                                 self.GRID_UNIT*self._zoom_multiplier))
            for world_coords in properties_dict["coords"]:
                screen_coords = self.world_to_screen(world_coords[0], world_coords[1], camera)
                # screen_coords[0] -= screen_coords[0] % 3
                # screen_coords[1] -= screen_coords[1] % 3
                if (-self.GRID_UNIT <= screen_coords[0] <= self.screen_width and
                        -self.GRID_UNIT <= screen_coords[1] <= self.screen_height):
                    self.surface.blit(image, (round(screen_coords[0]), round(screen_coords[1])))

    def save_tilemap(self):
        with open(self.mapfile, "w") as tiles:
            json.dump(self.tilemap, tiles, indent=1)

    def draw_grid(self, camera):
        start = self.world_to_screen(math.ceil(camera["x"]), math.ceil(camera["y"]), camera)
        start_x = start[0]
        start_y = start[1]

        x = start_x
        while x < self.screen_width:
            pygame.draw.line(self.surface, "black", (x, self.screen_height), (x, 0), width=1)
            x += self.GRID_UNIT*self._zoom_multiplier

        y = start_y
        while y < self.screen_height:
            pygame.draw.line(self.surface, "black", (self.screen_width, y), (0, y), width=1)
            y += self.GRID_UNIT*self._zoom_multiplier

    def get_int_coords(self, pos, camera):
        world_coords = self.screen_to_world(pos[0], pos[1], camera)
        world_coords = [math.floor(world_coords[0]), math.ceil(world_coords[1])]
        return world_coords

    def draw_tile_picker(self, picked_tile, game_surface):
        bottom = 50
        height = 100
        padding = 15
        num_tiles = len(self.tile_files)
        tile_size = self.GRID_UNIT
        tile_background = 10
        width = num_tiles*tile_size + (num_tiles+1)*padding
        myrect = pygame.Rect((self.screen_width-width)/2, self.screen_height-bottom-height, width, height)
        pygame.draw.rect(game_surface, "white", myrect, border_radius=10)
        for index, (tile_name, tile_properties) in enumerate(self.tile_files.items()):
            tile_file: str = tile_properties["path"]
            if tile_name == picked_tile:
                myrect2 = pygame.Rect((self.screen_width-width)/2 + index*tile_size+(index+1)*padding-tile_background/2,
                                      self.screen_height-bottom-height/2-tile_size/2-tile_background/2,
                                      tile_size+tile_background, tile_size+tile_background)
                pygame.draw.rect(game_surface, "grey", myrect2, border_radius=5)
            img = pygame.image.load(tile_file)
            image = pygame.transform.scale(img, (tile_size, tile_size))
            game_surface.blit(image, ((self.screen_width-width)/2 + index*tile_size+(index+1)*padding,
                                      self.screen_height-bottom-height/2-tile_size/2))
        return {"rect": myrect, "tile_size": tile_size, "padding": 15}

    def draw_tile(self, tile_name, new_coords: list):
        for tile_properties in self.tilemap.values():
            world_coords_list = tile_properties["coords"]
            for world_coords in world_coords_list:
                if new_coords[0] == world_coords[0] and new_coords[1] == world_coords[1]:
                    world_coords_list.remove(world_coords)
        if tile_name in self.tilemap.keys():
            self.tilemap[tile_name]["coords"].append(new_coords)
        else:
            self.tilemap[tile_name] = {
                "coords": [new_coords],
            }
            for property_name, property_value in self.tile_files[tile_name].items():
                if property_name == "path":
                    continue
                self.tilemap[tile_name][property_name] = property_value

    def erase_tile(self, erase_coords: list):
        for tile_properties in self.tilemap.values():
            world_coords_list = tile_properties["coords"]
            for world_coords in world_coords_list:
                if erase_coords[0] == world_coords[0] and erase_coords[1] == world_coords[1]:
                    world_coords_list.remove(world_coords)

    def point_in_tiles(self, point: list[float | int]):
        for tile_properties in self.tilemap.values():
            coords_list = tile_properties["coords"]
            for coords in coords_list:
                if coords[0] <= point[0] <= coords[0] + 1 and coords[1] >= point[1] >= coords[1] - 1:
                    return True
        return False

    def points_in_tiles(self, points: list[list[float | int]], range_x, range_y):
        coords_list2 = []
        for tile_properties in self.tilemap.values():
            coords_list = tile_properties["coords"]
            for coords in coords_list:
                if range_x[0] < coords[0] < range_x[1] and range_y[0] < coords[1] < range_y[1]:
                    coords_list2.append(coords)
        points_list = []
        for point in points:
            point_collides = False
            for coords in coords_list2:
                if coords[0] <= point[0] <= coords[0] + 1 and coords[1] >= point[1] >= coords[1] - 1:
                    point_collides = True
                    break
            points_list.append(point_collides)
        return points_list

    def get_point_properties(self, point: list[float | int]) -> dict[str, list[list[int]] | int | float] | None:
        for tile_properties in self.tilemap.values():
            coords_list = tile_properties["coords"]
            for coords in coords_list:
                if coords[0] <= point[0] <= coords[0] + 1 and coords[1] >= point[1] >= coords[1] - 1:
                    return tile_properties
        return None
