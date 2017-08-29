from PythonClient import *
import sys
import time
import msgpackrpc
import math
import threading
import numpy as np
import Tkinter as tk

TIME_STEP = 0.01

def generate_planar_evader_path(speed, duration, f, start_x, start_y, start_z):
    t = 0
    
    path = []
    x = start_x
    y = start_y
    
    path.append((x, y, start_z))
    
    while t < duration:
        angle = f(t)
        x_dot = speed * math.sin(angle)
        y_dot = speed * math.cos(angle)

        x += x_dot * TIME_STEP
        y += y_dot * TIME_STEP

        path.append((x, y, start_z))
        t += TIME_STEP

    return path

class DronePlayer(AirSimClient):
    CAMERA_VIEWS = {"first person": AirSimImageType.Scene,\
                    "depth": AirSimImageType.DepthMeters,\
                    "segmentation": AirSimImageType.Segmentation}
    TAKEOFF_MAX_TIME = 10 # maximum time for drone to reach
                          # takeoff altitude, in seconds
    
    def __init__(self, ip = "127.0.0.1", port = 41451):
        """ Initialises a player

        Parameters:
            * ip (str): the IP address of the AirSim client,
            default value of home
        """

        AirSimClient.__init__(self, ip, port)

    def start(self):
        #self._get_GPS_location()
        self.set_views()

        arm_result = self.loop_arm()
        if (arm_result < 0):
            print("Could not arm the drone. Exiting.")
            return

        takeoff_result = self.loop_takeoff()
        if (takeoff_result < 0):
            print("Could not take off successfully. Exiting.")
            return

        while True:
            self.fly_box(-7, 1, 10)
            pos = self.getPosition()
            print(pos)


            refly = raw_input("Fly box again? [y]/n: ")
            if refly.strip().lower() == 'n':
                break


        print("Hovering...")
        self.hover()

    def _get_GPS_location(self):
        """ Waits for GPS location to be set by AirSim server
        """
        
        print("Waiting for home GPS location to be set..."),
        
        home = self.getHomePoint()

        while ((home[0] == 0 and home[1] == 0 and home[2] == 0) or
               math.isnan(home[0]) or  math.isnan(home[1]) or \
               math.isnan(home[2])):
            print("..."),
            time.sleep(1)
            home = self.getHomePoint()

        self._home = home
        print("\nHome is:\n\tlatitude: {0:2f}, longitude: {1:2f}, altitude (m): {2:2f}"\
              .format(*tuple(home)))

    def set_views(self):
        """ Sets the camera views. If the user does not enter 'y', the view
        is off by default.
        """
        
        for name, cam_view in DronePlayer.CAMERA_VIEWS.iteritems():
            view_setting = raw_input("Would you like to turn the '{0}' view on? "\
                                        "y/[n]: ".format(name))

            if view_setting.strip().lower() == 'y':
                self.getImageForCamera(0, cam_view)

    def loop_arm(self):
        """ Attempts to arm the drone until successful, or until the user gives
        up by entering 'n'.
        """
        
        while True:
            print("Attempting to arm the drone...")
            if (not self.arm()):
                retry= raw_input("Failed to arm the drone. Retry? [y]/n: ")

                if retry.strip().lower() == 'n':
                    return -1
            else:
                break

        return 0

    def loop_takeoff(self):
        """ Attempts to take off until successful, or until the user gives up by
        entering 'n'.
        """
        
        if (self.getLandedState() == LandedState.Landed):
            print("Taking off...")

            while True:
                try:
                    self.takeoff(DronePlayer.TAKEOFF_MAX_TIME) 

                    print("Should now be flying...")
                    break

                except:

                    retry = raw_input("Failed to reach takeoff altitude after \
                            {0} seconds. Retry? [y]/n: "\
                            .format(DronePlayer.TAKEOFF_MAX_TIME))

                    if retry.strip().lower() == 'n':
                        return -1
                    
        else:
            print("It appears the drone is already flying")

        return 0

    def fly_box(self, height, speed, side_length):
        """ Makes the drone fly in a box at the specified height and speed.
        Hovers once the box has been flown

        Parameters:
            * height (float): height above the original launch point, in m
                                Note: AirSim uses NED coordinates
                                        so negative axis is up.
            * speed (float): speed to fly at, in m/s
            * side_length (float): side length of box, in m
        """
        
        direction_vectors = [(1, 0), (0, 1), (-1, 0), (0, -1)]

        duration = float(side_length) / speed
        delay = duration * speed

        for x_dir, y_dir in direction_vectors:
            x_velocity = x_dir * speed
            y_velocity = y_dir * speed

            yaw_angle = math.degrees(math.atan2(y_dir, x_dir))

            self.moveByVelocityZ(x_velocity, y_velocity, height, duration,\
                    DrivetrainType.MaxDegreeOfFreedom, YawMode(False, yaw_angle))
            time.sleep(delay)

        self.hover()
            
        

class PlayerModel(object):
    """ Attributes of a player
    """
    PLAYER_COLOURS = ["blue", "red", "green", "yellow"]
    colour_index = 0
    def __init__(self, speed, start_pos = (0, 0, 0)):
        """ Initialises player attributes

        Parameters:
            * start_pos (tup<float, float>): start position in Cartesian coords
                                            (x, y, z), in m
        """
        self._start_pos = start_pos
        self._x, self._y, self._z = self._start_pos
        self._position_log = [self._start_pos]

        self._speed = speed

        self._colour = PlayerModel.PLAYER_COLOURS[PlayerModel.colour_index]
        PlayerModel.colour_index = (PlayerModel.colour_index + 1) % len(PlayerModel.PLAYER_COLOURS)

    def get_speed(self):
        return self._speed

    def get_colour(self):
        return self._colour

    def record_current_pos(self):
        """ Records current position in position log
        """
        self._position_log.append((self._x, self._y, self._z))

    def reset_position_log(self):
        """ Resets position log
        """
        self._position_log = [self._start_pos]

    def get_position_log(self):
        return self._position_log[:]

##    def get_next_pos(self):
##        self._pos_index += 1
##        
##        if self._pos_index == len(self._position_log):
##            return None
##
##        return self._position_log[self._pos_index]

    def move(self, pos):
        self._x, self._y, self._z = pos
        self.record_current_pos()

    def get_current_pos(self):
        return self._position_log[-1]

    def get_most_recent_segment(self):
        if len(self._position_log) < 2:
            return

        return (self._position_log[-2], self._position_log[-1])

class Evader(PlayerModel):
    def __init__(self, speed, start_pos = (0, 5, 0)):
        PlayerModel.__init__(self, speed, start_pos)

        self._path = []

    def set_path(self, path):
        self._path = path
        self._path_index = -1

    def pre_determined_move(self):
        self._path_index += 1

        if (self._path_index == len(self._path)):
            return True

        self.move(self._path[self._path_index])


class Pursuer(PlayerModel):
    def __init__(self, speed, min_turn_r, start_pos = (0, 0, 0)):
        PlayerModel.__init__(self, speed, start_pos)

        self._R = min_turn_r
        self._capture_r = None
        self._prey = None
        self._angle = 0
    
    def set_prey(self, evader, capture_radius):
        self._prey = evader
        self._capture_r = capture_radius

    def chase(self):
        if self._prey == None:
            return True

        target_pos = self._prey.get_current_pos()
        current_pos = self.get_current_pos()

        x = (target_pos[0] - current_pos[0]) * math.cos(self._angle) \
                - (target_pos[1] - current_pos[1]) * math.sin(self._angle)
        y = (target_pos[0] - current_pos[0]) * math.sin(self._angle) \
                + (target_pos[1] - current_pos[1]) * math.cos(self._angle)

        d = math.sqrt(x**2 + y**2)

        if d < self._capture_r:
            return True

        if (x == 0):
            if (y < 0):
                phi = 1
            else:
                phi = 0
        else:
            phi = np.sign(x)

        angle_dot = float( self._speed * phi ) / self._R
        self._angle += angle_dot * TIME_STEP

        p_x_dot = self._speed * math.sin(self._angle)
        p_y_dot = self._speed * math.cos(self._angle)

        p_x = current_pos[0] + p_x_dot * TIME_STEP
        p_y = current_pos[1] + p_y_dot * TIME_STEP

        self.move((p_x, p_y, current_pos[2]))

        return False

        
class PositionPlotter(tk.Canvas):
    WIDTH = 500
    HEIGHT = 1000
    MAX_X = 5
    MIN_X = -5
    MAX_Y = 20
    MIN_Y = 0
    def __init__(self, parent):
        tk.Canvas.__init__(self, parent, bg = "grey", width=PositionPlotter.WIDTH, height=PositionPlotter.HEIGHT)
        self.pack(expand = True, fill = tk.BOTH)

        self._m_x = float(PositionPlotter.WIDTH) / (PositionPlotter.MAX_X - PositionPlotter.MIN_X)
        self._c_x = -self._m_x * PositionPlotter.MIN_X
        
        self._m_y = float(PositionPlotter.HEIGHT) / (PositionPlotter.MAX_Y - PositionPlotter.MIN_Y)
        self._c_y = -self._m_y * PositionPlotter.MIN_Y

    def scale_x(self, x):
        return round(self._m_x * x + self._c_x)

    def scale_y(self, y):
        return round(self._m_y * y + self._c_y)

    def planar_scale(self, x, y):
        return ( self.scale_x(x), self.scale_y(y) )


    def plot_path(self, points):
        planar_points = []

        for x, y, z in points:          
            planar_points.append(self.planar_scale(x, y))

        for i in range(len(planar_points[:-1])):
            self._canvas.create_line(planar_points[i], planar_points[i+1])


    def update_plot(self, player):
        path_segment = player.get_most_recent_segment()
        
        col = player.get_colour()

        if path_segment:
            start, end = path_segment
            scaled_start = self.planar_scale(*start[:-1])
            scaled_end = self.planar_scale(*end[:-1])
            self.create_line(scaled_start, scaled_end, fill = col)

            

##        if not next_pos:
##            return
##
##        x, y, z = next_pos
##        scaled = self.planar_scale(x, y)
##        
##        if self._last_pos:
##            self.create_line(self._last_pos, scaled)
##
##        self._last_pos = scaled
##    def demo(self, e):
##        p = PlayerModel()
##        p.planar_sinusoidal_swerve(2, 2*math.pi, 3)
##
##        coords = p.get_position_log()
##        print(coords[:10])
##
##        print("Swerved")
##
##        
##        self.plot_path(coords)


class PEGameApp(object):
    def __init__(self, master):
        self._master = master
        self._master.title("Position plotter application")

        self._plotter = PositionPlotter(self._master)

        self._evader = Evader(0.5)

        self._pursuer = Pursuer(1, 1)
        self._pursuer.set_prey(self._evader, 0.01)

        evader_start_pos = self._evader.get_current_pos()
        evader_speed = self._evader.get_speed()
        swerve_path = generate_planar_evader_path(evader_speed, 60, lambda x: math.cos(x), *evader_start_pos)
        self._evader.set_path(swerve_path)
        self.chase()

    def chase(self):
        path_finished = self._evader.pre_determined_move()
        caught = self._pursuer.chase()
        self._plotter.update_plot(self._evader)
        self._plotter.update_plot(self._pursuer)

        if (path_finished):
            return

        if (caught):
            return
        
        self._master.after(1, self.chase)

if __name__ == "__main__":
    drone = DronePlayer("10.33.68.87", 41451)
    drone.start()
                

##    root = tk.Tk()
##    app = PEGameApp(root)
##    root.mainloop()
        
