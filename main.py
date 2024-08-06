import math
import time
import pygame
import sys
from tilemap_manager import Tilemap


class Player:
    def __init__(self, *, pos: list = None, velo: list = None, spawnpoint: list = None, horizontal_acc=24,
                 max_horizontal_velo=6, horizontal_deceleration=20, horizontal_slow_deceleration=5, jump_velo=12,
                 gravity=30, min_vert_velo=17, health=1, extra_jumps=0, jump_buffer=0.1, coyote_time=0.1,
                 climbing_acc=24, climbing_max_velo=6, wall_jump_velo: list = None, wall_max_distance=0.1,
                 climbing_deceleration=20, no_input_time=0.2):
        if pos is None:
            pos = [0, 0]
        if velo is None:
            velo = [0, 0]
        if spawnpoint is None:
            spawnpoint = [0, 1]
        if wall_jump_velo is None:
            wall_jump_velo = [6, 12]

        self.pos = pos
        self.velo = velo
        self.horizontal_acc = horizontal_acc
        self.max_horizontal_velo = max_horizontal_velo
        self.horizontal_deceleration = horizontal_deceleration
        self.horizontal_slow_deceleration = horizontal_slow_deceleration
        self.gravity = gravity
        self.jump_velo = jump_velo
        self.extra_jumps_left = 0
        self.extra_jumps = extra_jumps
        self.last_jump_time = -5
        self.jump_buffer = jump_buffer
        self.last_grounded_time = 0
        self.coyote_time = coyote_time
        self.grounded = False
        self.ceil_col = False
        self.left_col = False
        self.right_col = False
        self.show_collider = False
        self.wheel_spin = 0
        self.last_wheel_spin = 0
        self.frame = 0
        self.elasticity = 0.2
        self.last_pos = pos
        self.min_vert_velo = min_vert_velo
        self.health = health
        self.spawnpoint = spawnpoint
        self.collider_points = 40
        self.is_climbing = False
        self.climbing_acc = climbing_acc
        self.climbing_max_velo = climbing_max_velo
        self.last_horizontal_speed = 0
        self.wall_max_distance = wall_max_distance
        self.wall_jump_velo = wall_jump_velo
        self.climbing_deceleration = climbing_deceleration
        self.no_input_time = no_input_time
        self.last_wall_jump = -50

    def reset(self):
        self.velo = [0, 0]
        self.pos = self.spawnpoint.copy()
        self.is_climbing = False
        self.last_wall_jump = -50
        self.wheel_spin = 0
        self.last_wheel_spin = 0
        self.frame = 0
        self.health = 1
        self.grounded = False

    def die(self):
        self.reset()

    def jump(self):
        self.velo[1] = self.jump_velo

    def wall_jump(self, wall_side):
        self.velo = self.wall_jump_velo.copy()
        self.velo[0] *= -wall_side
        self.last_wall_jump = time.perf_counter()

    def handle_jump(self, current_frame_input, collides: dict):
        if current_frame_input["up"] == 1:
            self.last_jump_time = time.perf_counter()
        jump_input = current_frame_input["up"] == 1 or time.perf_counter() - self.last_jump_time <= self.jump_buffer
        if self.grounded:
            self.last_grounded_time = time.perf_counter()
            self.extra_jumps_left = self.extra_jumps
            ground_check = True
        elif time.perf_counter() - self.last_grounded_time < self.coyote_time and self.velo[1] <= 0:
            self.extra_jumps_left = self.extra_jumps
            ground_check = True
        else:
            ground_check = False

        if collides["left"]:
            wall_side = -1
            wall_check = True
        elif collides["right"]:
            wall_side = 1
            wall_check = True
        else:
            wall_check = False

        if jump_input:
            if ground_check:
                self.last_jump_time -= self.jump_buffer
                self.jump()
            elif wall_check:
                self.last_jump_time -= self.jump_buffer
                self.wall_jump(wall_side)
            elif self.extra_jumps_left > 0:
                self.extra_jumps_left -= 1
                self.last_jump_time -= self.jump_buffer
                self.jump()

    def draw_collider(self, tiles: Tilemap, surface, camera_coords, color):
        rad = 2 * math.pi / self.collider_points
        for i in range(self.collider_points):
            point = [(math.cos(i * rad) + 1) / 2 + self.pos[0], (math.sin(i * rad) - 1) / 2 + self.pos[1]]
            screen_point = tiles.world_to_screen(point[0], point[1], camera_coords)
            pygame.draw.circle(surface=surface, center=screen_point, color=color, radius=2)

    def draw_player(self, size, frames_files, camera_coords, tiles: Tilemap, surface):
        img = pygame.image.load(frames_files[self.frame])
        image = pygame.transform.scale(img, (size, size))
        img_pos = tiles.world_to_screen(self.pos[0], self.pos[1], camera_coords)
        # img_pos[0] -= img_pos[0] % 3
        # img_pos[1] -= img_pos[1] % 3
        surface.blit(image, img_pos)
        if self.show_collider:
            self.draw_collider(tiles, surface, camera_coords, (200, 200, 200))

    @staticmethod
    def is_colliding(collides: list[bool]):
        for point_collide in collides:
            if point_collide:
                return True
        return False

    def update(self, tiles: Tilemap, dtime: float, game_input, current_frame_input, surface, camera, step_size=0.005):
        if self.show_collider:
            self.draw_collider(tiles, surface, camera, (90, 90, 90))

        collides = self.check_main_collides(tiles, radius_extra=self.wall_max_distance)
        self.handle_jump(current_frame_input, collides)

        horizontal_direction = game_input["right"] - game_input["left"]
        velo_hori_direction = math.copysign(1, self.velo[0])
        if (time.perf_counter() - self.last_wall_jump < self.no_input_time and
                not velo_hori_direction == horizontal_direction):
            horizontal_direction = 0
        horizontal_acc = horizontal_direction * self.horizontal_acc
        velo_diff = [0, 0]
        self.is_climbing = False
        if horizontal_acc == 0 and self.velo != 0:
            if time.perf_counter() - self.last_wall_jump < self.no_input_time:
                velo_diff[0] = self.horizontal_slow_deceleration * dtime * -velo_hori_direction
            else:
                velo_diff[0] = self.horizontal_deceleration * dtime * -velo_hori_direction
            if abs(velo_diff[0]) > abs(self.velo[0]):
                velo_diff[0] = -self.velo[0]
        elif (horizontal_direction == 1 and self.right_col) or (horizontal_direction == -1 and self.left_col):
            self.velo[1] = max(self.velo[1], self.last_horizontal_speed)
            velo_diff[1] = self.climbing_acc * dtime
            self.is_climbing = True
        else:
            velo_diff[0] = horizontal_acc * dtime

        if (velo_diff[0] > 0 and self.right_col) or (velo_diff[0] < 0 and self.left_col):
            velo_diff[0] = 0

        if -self.max_horizontal_velo <= self.velo[0] <= self.max_horizontal_velo:
            self.velo[0] += velo_diff[0]
            self.velo[0] = min(self.velo[0], self.max_horizontal_velo)
            self.velo[0] = max(self.velo[0], -self.max_horizontal_velo)
        elif (self.velo[0] > self.max_horizontal_velo and horizontal_direction >= 0) or \
                (self.velo[0] < -self.max_horizontal_velo and horizontal_direction <= 0):
            velo_diff[0] = - math.copysign(self.horizontal_slow_deceleration*dtime, self.velo[0])
            self.velo[0] += velo_diff[0]
        else:
            velo_diff[0] = horizontal_acc * dtime
            self.velo[0] += velo_diff[0]

        if not self.grounded and not self.is_climbing:
            velo_diff[1] -= self.gravity * dtime
        if self.climbing_max_velo < self.velo[1] and self.is_climbing:
            self.velo[1] -= self.climbing_deceleration * dtime
        else:
            self.velo[1] += velo_diff[1]
            if self.is_climbing:
                self.velo[1] = min(self.velo[1], self.climbing_max_velo)
        self.velo[1] = max(self.velo[1], -self.min_vert_velo)

        pos_diff = [self.velo[0] * dtime, self.velo[1] * dtime]
        new_pos = [self.pos[0] + pos_diff[0], self.pos[1] + pos_diff[1]]
        collides = self.check_collides(tiles, pos=new_pos)
        velo_magnitude = math.sqrt(self.velo[0]*self.velo[0] + self.velo[1]*self.velo[1])

        if velo_magnitude == 0:
            return

        self.last_horizontal_speed = abs(self.velo[0])
        velo_normal = [self.velo[0]/velo_magnitude, self.velo[1]/velo_magnitude]
        step = [velo_normal[0]*step_size, velo_normal[1]*step_size]
        step_time = step_size/velo_magnitude
        time_travelled = dtime
        self.last_pos = new_pos
        if Player.is_colliding(collides) and self.is_climbing:
            if self.left_col:
                new_pos[0] += 0.02
            else:
                new_pos[0] -= 0.02
            collides = self.check_collides(tiles, pos=new_pos)
        while Player.is_colliding(collides):
            self.last_pos = new_pos
            new_pos = [new_pos[0] - step[0], new_pos[1] - step[1]]
            time_travelled -= step_time
            collides = self.check_collides(tiles, pos=new_pos)

        time_left = dtime - time_travelled
        self.health -= self.get_colliding_danger(tiles, self.last_pos)
        if self.health <= 0:
            self.die()
            return
        new_velo = self.velo_after_collides(pos=self.last_pos, tiles=tiles, update_cols=True)
        new_pos = [new_pos[0]+new_velo[0] * time_left, new_pos[1]+new_velo[1] * time_left]
        if self.grounded:
            self.wheel_spin += pos_diff[0]
            self.last_wheel_spin = pos_diff[0]
        elif self.left_col:
            self.wheel_spin -= pos_diff[1]
            self.last_wheel_spin = -pos_diff[1]
        elif self.right_col:
            self.wheel_spin += pos_diff[1]
            self.last_wheel_spin = pos_diff[1]
        elif self.ceil_col:
            self.wheel_spin -= pos_diff[0]
            self.last_wheel_spin = -pos_diff[0]
        else:
            self.wheel_spin += self.last_wheel_spin
        self.frame = round(self.wheel_spin / math.pi * 8) % 8
        self.pos = new_pos
        self.velo = new_velo

    def velo_after_collides(self, pos: list, tiles: Tilemap, new_velo: list = None, elasticity=1,
                            min_corner_elasticity=0.7, adjust_dir=0.2, update_cols=False):
        if new_velo is None:
            new_velo = self.velo
        points = self.check_collides(tiles, pos=pos)
        rad = 2 * math.pi / len(points)

        # update colliders
        colliders = self.check_main_collides(tiles, pos)
        dots = {
            "left": [-0.1, -0.5],
            "right": [1 + 0.1, -0.5],
            "up": [0.5, 0.1],
            "down": [0.5, -1 - 0.1],
        }
        for point in dots.values():
            point[0] += self.pos[0]
            point[1] += self.pos[1]
        if update_cols:
            if not colliders["down"] or not new_velo[1] == 0:
                self.grounded = False
            elif tiles.get_point_properties(dots["down"]) and tiles.get_point_properties(dots["down"])["danger"] > 0:
                self.grounded = False
            if not colliders["up"] or not new_velo[1] == 0:
                self.ceil_col = False
            elif tiles.get_point_properties(dots["up"]) and tiles.get_point_properties(dots["up"])["danger"] > 0:
                self.ceil_col = False
            if not colliders["left"] or not new_velo[0] == 0:
                self.left_col = False
            elif tiles.get_point_properties(dots["left"]) and tiles.get_point_properties(dots["left"])["danger"] > 0:
                self.left_col = False
            if not colliders["right"] or not new_velo[0] == 0:
                self.right_col = False
            elif tiles.get_point_properties(dots["right"]) and tiles.get_point_properties(dots["right"])["danger"] > 0:
                self.right_col = False

        for i, point_bool in enumerate(points):
            if point_bool:
                point_rad = i*rad
                velo_rad = math.atan2(new_velo[1], new_velo[0])
                new_rad = math.pi - velo_rad + 2*point_rad
                velo_magnitude = math.sqrt(new_velo[0]*new_velo[0] + new_velo[1]*new_velo[1])
                rad_diff = abs((point_rad % (2*math.pi)) - (velo_rad % (2*math.pi)))
                rad_diff = min(rad_diff, 2*math.pi-rad_diff)

                if rad_diff >= math.pi/2:
                    rad_adjust = abs(point_rad - rad_diff)*adjust_dir
                    rad_adjust = min(rad_adjust, 2*math.pi-rad_adjust)
                    rad_diff1 = new_rad % (2*math.pi) - (math.pi+point_rad) % (2*math.pi)
                    clockwise = math.copysign(1, rad_diff1) * math.copysign(1, abs(rad_diff1)-math.pi)
                    new_rad += clockwise * rad_adjust

                dir_x = math.cos(new_rad)
                dir_y = math.sin(new_rad)

                if update_cols:
                    if point_rad == math.pi/2:
                        self.ceil_col = True
                    elif point_rad == math.pi*3/2:
                        self.grounded = True
                    elif point_rad == math.pi:
                        self.left_col = True
                    elif point_rad == 0:
                        self.right_col = True

                point = [(math.cos(i * rad) + 1) / 2 + pos[0], (math.sin(i * rad) - 1) / 2 + pos[1]]
                bounciness: float = tiles.get_point_properties(point)["bounciness"]
                if rad_diff >= math.pi/2:
                    new_velo = [velo_magnitude*dir_x, velo_magnitude*dir_y]
                elif point_rad % math.pi == math.pi/2:
                    new_velo = [velo_magnitude*dir_x, bounciness*elasticity*velo_magnitude*dir_y]
                    if abs(new_velo[1]) < 2:
                        new_velo[1] = 0
                elif point_rad % math.pi == 0:
                    new_velo = [bounciness*elasticity*velo_magnitude*dir_x, velo_magnitude*dir_y]
                    if abs(new_velo[0]) < 2:
                        new_velo[0] = 0
                else:
                    min_corner_elasticity = max(min_corner_elasticity, bounciness)
                    new_velo = [min_corner_elasticity * velo_magnitude * dir_x,
                                min_corner_elasticity * velo_magnitude * dir_y]
        new_velo[0] = round(new_velo[0], 4)
        new_velo[1] = round(new_velo[1], 4)
        return new_velo

    def check_main_collides(self, tiles: Tilemap, pos: list = None, radius=0.5, radius_extra=0.1) -> dict[str, bool]:
        if pos is None:
            pos = self.pos

        points = {
            "left": [-radius_extra, -radius],
            "right": [1+radius_extra, -radius],
            "up": [radius, radius_extra],
            "down": [radius, -1-radius_extra],
        }
        points_collided: dict[str, bool] = {
        }
        for dir_, point in points.items():
            point[0] += pos[0]
            point[1] += pos[1]
            collided = tiles.point_in_tiles(point)
            points_collided[dir_] = collided
        return points_collided

    def check_collides(self, tiles: Tilemap, pos: list = None):
        if pos is None:
            pos = self.pos

        rad = 2 * math.pi / self.collider_points
        points_list = []
        for i in range(self.collider_points):
            point = [(math.cos(i * rad) + 1) / 2 + pos[0], (math.sin(i * rad) - 1) / 2 + pos[1]]
            points_list.append(point)
        return tiles.points_in_tiles(points_list, [self.pos[0]-2, self.pos[0]+2], [self.pos[1]-2, self.pos[1]+2])

    def get_collided_tiles_properties(self, tiles: Tilemap, pos: list = None):
        if pos is None:
            pos = self.pos

        points_properties = []
        rad = 2 * math.pi / self.collider_points
        for i in range(self.collider_points):
            point = [(math.cos(i * rad) + 1) / 2 + pos[0], (math.sin(i * rad) - 1) / 2 + pos[1]]
            props = tiles.get_point_properties(point)
            if props is not None:
                points_properties.append(props)
        return points_properties

    def get_colliding_danger(self, tiles: Tilemap, pos: list = None) -> int:
        if pos is None:
            pos = self.pos

        total_danger = 0
        properties = self.get_collided_tiles_properties(tiles, pos=pos)
        for tile_properties in properties:
            total_danger += tile_properties["danger"]
        return total_danger


def main():
    BLUE = 50, 80, 150
    GRID_UNIT = 48
    background_color = BLUE
    pygame.init()
    SCREEN_WIDTH = pygame.display.Info().current_w
    SCREEN_HEIGHT = pygame.display.Info().current_h
    camera_offset = [-SCREEN_WIDTH/(2*GRID_UNIT), 1+SCREEN_HEIGHT/(2*GRID_UNIT)]
    camera = {"x": camera_offset[0], "y": camera_offset[1]}
    surface = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    pygame.display.set_caption("Wheel Game")
    clock = pygame.time.Clock()
    mapfile = "map.json"
    tile_files = {
        "grass": {"path": "grass.png"},
        "dirt": {"path": "dirt.png"},
        "bouncy": {"path": "bouncy.png"},
        "wave": {"path": "wave1.png"},
        "water": {"path": "water1.png"},
        "invis": {"path": "invis.png"},
    }
    wheel_files = [f"wheel{frame}.png" for frame in range(1, 9)]

    tilemap = Tilemap(
        mapfile,
        tile_files,
        surface,
        GRID_UNIT,
    )

    player = Player()

    move_input: dict[str, bool] = {
        "up": False,
        "down": False,
        "right": False,
        "left": False,
    }

    frame_input: dict[str, int] = {
        "up": 0,
        "down": 0,
        "right": 0,
        "left": 0,
    }
    # -1 = released, 0 = hadn't changed, 1 = pressed

    while True:
        surface.fill(background_color)
        tilemap.draw_map(camera)

        # Input

        frame_input["up"] = 0
        frame_input["down"] = 0
        frame_input["right"] = 0
        frame_input["left"] = 0

        for event in pygame.event.get():
            e = event.dict
            if event.type == pygame.QUIT:
                tilemap.save_tilemap()
                pygame.quit()
                sys.exit()
            if event.type == pygame.KEYDOWN:
                if e["key"] == 27:  # escape
                    tilemap.save_tilemap()
                    pygame.quit()
                    sys.exit()
                if e["key"] == 1073741904 or e["key"] == 97:  # left
                    move_input["left"] = True
                    frame_input["left"] = 1
                if e["key"] == 1073741906 or e["key"] == 119:  # up
                    move_input["up"] = True
                    frame_input["up"] = 1
                if e["key"] == 1073741903 or e["key"] == 100:  # right
                    move_input["right"] = True
                    frame_input["right"] = 1
                if e["key"] == 1073741905 or e["key"] == 115:  # down
                    move_input["down"] = True
                    frame_input["down"] = 1
            if event.type == pygame.KEYUP:
                if e["key"] == 1073741904 or e["key"] == 97:  # left
                    move_input["left"] = False
                    frame_input["left"] = -1
                if e["key"] == 1073741906 or e["key"] == 119:  # up
                    move_input["up"] = False
                    frame_input["up"] = -1
                if e["key"] == 1073741903 or e["key"] == 100:  # right
                    move_input["right"] = False
                    frame_input["right"] = -1
                if e["key"] == 1073741905 or e["key"] == 115:  # down
                    move_input["down"] = False
                    frame_input["down"] = -1

        # Updating Player
        dt = clock.tick() / 1000
        player.update(tilemap, min(dt, 0.1), move_input, frame_input, surface, camera)
        last_camera_pos = [camera["x"]-camera_offset[0], camera["y"]-camera_offset[1]]
        catch_up_distance = [player.pos[0] - last_camera_pos[0], player.pos[1] - last_camera_pos[1]]
        new_camera_pos = [last_camera_pos[0] + catch_up_distance[0]/20, last_camera_pos[1] + catch_up_distance[1]/20]
        # camera = {"x": new_camera_pos[0]+camera_offset[0], "y": new_camera_pos[1]+camera_offset[1]}
        camera = {"x": player.pos[0]+camera_offset[0], "y": player.pos[1]+camera_offset[1]}
        player.draw_player(GRID_UNIT, wheel_files, camera, tilemap, surface)

        # print(f"{player.pos = }")
        # print(f"{player.velo = }")

        pygame.display.update()


if __name__ == '__main__':
    main()
