import board
import adafruit_gps
import adafruit_tca9548a
import time
from typing import Literal

# GPS initialization commands
GGA_RMC_COMMAND = b"PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0"
UPDATE_RATE_COMMAND = b"PMTK220,"

# GPS default update rate in milliseconds
DEFAULT_UPDATE_RATE_MS = 1000
# GPS fix attempt limit
GPS_FIX_ATTEMPT_LIMIT = 10

# Allowed multiplexer port numbers
allowed_ports = Literal[0, 1, 2, 3, 4, 5, 6, 7]


class GPS:
    def __init__(self, gps_port: allowed_ports, update_rate_ms: int = DEFAULT_UPDATE_RATE_MS):
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

        # get GPS fix
        self._get_fix()

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
        while (not self.GPS.has_fix) and (attempt_count < GPS_FIX_ATTEMPT_LIMIT):
            # if not, wait and check again
            time.sleep(1)
            self.GPS.update()
            attempt_count += 1

        return attempt_count < GPS_FIX_ATTEMPT_LIMIT

    def get_position(self):
        """
        Gets the current GPS position
        :return: If the GPS has a fix, a list containing the GPS position in the following order:
        [latitude (deg), longitude (deg), altitude (m)]. Otherwise, None
        """
        # update GPS
        self.GPS.update()

        # make sure GPS has fix
        if not self._get_fix():
            return None

        # get position as list: [latitude, longitude, altitude]
        position = [self.GPS.latitude, self.GPS.longitude, self.GPS.altitude_m]
        return position
