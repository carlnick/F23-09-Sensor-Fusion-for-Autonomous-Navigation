import board
import adafruit_gps
import adafruit_tca9548a
import time
from typing import Literal
from math import pi
import math
import numpy as np

EARTH_RADIUS_METERS = 6371000

# GPS initialization commands
GGA_RMC_COMMAND = b"PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n"
UPDATE_RATE_COMMAND = b"PMTK220,"

# GPS default update rate in milliseconds
DEFAULT_UPDATE_RATE_MS = 1000
# GPS fix attempt limit
GPS_FIX_ATTEMPT_LIMIT = 10

# Allowed multiplexer port numbers
allowed_ports = Literal[0, 1, 2, 3, 4, 5, 6, 7]


class GPS:
    def __init__(self, gps_port: int = 5, update_rate_ms: int = DEFAULT_UPDATE_RATE_MS):
        """
        Wrapper class for the Adafruit PA1010D GPS module
        :param gps_port: The port on the multiplexer that the GPS is connected to
        :param update_rate_ms: The update rate of the GPS in milliseconds (default: 1000)
        """
        # initialize multiplexer
        mux = adafruit_tca9548a.TCA9548A(board.I2C())

        # initialize GPS from port on multiplexer
        self.GPS = adafruit_gps.GPS_GtopI2C(mux[gps_port])

        # send configuration command, GPS will report:
        # GPGGA interval - GPS Fix Data
        # GPRMC interval - Recommended Minimum Specific GNSS Sentence
        self.GPS.send_command(GGA_RMC_COMMAND)

        # send update rate command according to update rate
        self.GPS.send_command(UPDATE_RATE_COMMAND + str(update_rate_ms).encode())

        # initialize GPS position in degrees
        self.position_degrees = None

        # get GPS fix
        self._get_fix()

        # get GPS position
        self._get_position()

    def _get_fix(self) -> bool:
        """
        Attempts to get a GPS fix, times out after GPS_FIX_ATTEMPT_LIMIT attempts
        :return: True if the GPS has a fix, False otherwise
        """
        # update GPS
        self.GPS.update()

        # keep track of attempts
        attempt_count = 0

        # check if GPS has fix
        while ((not self.GPS.has_fix) and (attempt_count < GPS_FIX_ATTEMPT_LIMIT)):
            # if not, wait and check again
            print("GPS could not get a fix - attempting to retry")
            time.sleep(0.1)
            self.GPS.update()
            attempt_count += 1

        return attempt_count < GPS_FIX_ATTEMPT_LIMIT

        #return self.GPS.has_fix

    def get_position_meters(self):
        """
        Gets the difference between the current GPS position and the initial GPS position (in meters)
        :return: The difference between the current GPS position and the initial GPS position as a list [x, y, z]
        """
        self._get_fix()

        current_position = [self.GPS.latitude, self.GPS.longitude, self.GPS.altitude_m]

        # Distance away between current GPS position and initial GPS position
        latitude_difference = current_position[0] - self.position_degrees[0]
        longitude_difference = current_position[1] - self.position_degrees[1]
        altitude_difference = current_position[2] - self.position_degrees[2]

        x_difference_mtrs = self._deg_to_m(latitude_difference)
        y_difference_mtrs = self._deg_to_m(longitude_difference)
        z_difference_mtrs = altitude_difference

        dist_mtrs = math.sqrt(x_difference_mtrs**2 + y_difference_mtrs**2) 
        azimuth = np.arctan2(y_difference_mtrs, x_difference_mtrs)

        return x_difference_mtrs, y_difference_mtrs, z_difference_mtrs, dist_mtrs, azimuth

    def _get_position(self):
        if (not self._get_fix()):
            return None
        else:
            self.position_degrees = [self.GPS.latitude, self.GPS.longitude, self.GPS.altitude_m]
            return self.position_degrees

    def _deg_to_m(self, deg):
        return (2.0 * pi * EARTH_RADIUS_METERS * deg) / 360.0

    @staticmethod
    def change_in_position_between_two_points(lat_1, lon_1, lat_2, lon_2):
        '''Get distance between two GPS points'''
        
        lat_1_rad = np.deg2rad(lat_1)
        lon_1_rad = np.deg2rad(lon_1)
        lat_2_rad = np.deg2rad(lat_2)
        lon_2_rad = np.deg2rad(lon_2)

        delta_lat = lat_2_rad - lat_1_rad
        delta_lon = lon_2_rad - lon_1_rad

        a = np.sin(delta_lat / 2.0)**2 + np.cos(lat_1_rad) * np.cos(lat_2_rad) * np.sin(delta_lon / 2.0)**2
        c = 2 * np.arctan2(np.sqrt(a), np.sqrt(1 - a))

        return c * EARTH_RADIUS_METERS


    @staticmethod
    def latToMtrs(latitude):
        return change_in_position_between_two_points(latitude, 0.0, 0.0, 0.0)

    @staticmethod
    def lonToMtrs(longitude):
        return change_in_position_between_two_points(0.0, longitude, 0.0, 0.0)

        
