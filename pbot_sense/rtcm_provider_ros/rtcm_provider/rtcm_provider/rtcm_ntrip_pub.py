import rclpy
from rclpy.node import Node
from rtcm_msgs.msg import Message as Rtcm
from sensor_msgs.msg import NavSatFix
from ament_index_python.packages import get_package_share_directory

import os
from queue import Queue
from pyubx2 import RTCM3_PROTOCOL, protocol

from pygnssutils import VERBOSITY_LOW, GNSSNTRIPClient
import haversine

import yaml

# NTRIP caster parameters - AMEND AS REQUIRED:
# Ideally, mountpoint should be <30 km from location.
IPPROT = "IPv4"  # or "IPv6"
NTRIP_SERVER = "www.gnssdata.or.kr"
NTRIP_PORT = 2101
FLOWINFO = 0  # for IPv6
SCOPEID = 0  # for IPv6
MOUNTPOINT = "TEGN-RTCM32"  # leave blank to retrieve sourcetable
NTRIP_USER = "kde1054@naver.com"
NTRIP_PASSWORD = "gnss"
# NMEA GGA sentence status - AMEND AS REQUIRED:
GGAMODE = 0  # use fixed reference position (0 = use live position)
GGAINT = -1  # interval in seconds (-1 = do not send NMEA GGA sentences)

# CHECK_COORD_TIMER_PERIOD = 60  # seconds
REBASE_CHECK_COUNT = 400 # number of rtcm messages to check if rebase is needed or not
REBASE_DISTANCE_THRESHOLD = 30  # km

def ddmmss_to_decdd(ddmmss: str) -> float:
    """Converts a string in the format ddmmss to decimal degrees."""
    split = ddmmss.split("-")
    dd = float(split[0])
    mm = float(split[1])
    ss = float(split[2])
    return dd + mm / 60 + ss / 3600

class RtcmNtripPub(Node):
    def __init__(self):
        super().__init__('rtcm_ntrip_pub')

        self.ntrip_queue = Queue()

        self.rtcm_pub = self.create_publisher(Rtcm, '/rtcm', 10)
        self.gnss_sub = self.create_subscription(NavSatFix, '/gps/fix', self.onGnssSubCallBack, 1)

        self.rebase_check = False

        self.rtcmpub_timer = self.create_timer(0.1, self.onRtcmPubTimerCallBack)
        self.rtcmpub_timer.cancel()

        self.rtcm_cnt = 0
        self.current_mountpoint = None

        # Load RTCM base coordinates from config file
        with open(os.path.join(get_package_share_directory(__package__), 'korea_rtcm_base.yaml')) as f:
            self.rtcm_base_coords = yaml.load(f, Loader=yaml.FullLoader)
        
        for key, value in self.rtcm_base_coords['RTCM32-Base'].items():
            value[0], value[1] = ddmmss_to_decdd(value[0]), ddmmss_to_decdd(value[1])

        
        self.gnc = GNSSNTRIPClient(None, verbosity=VERBOSITY_LOW)
        self.streaming = None


    # @staticmethod
    def reselect_mountpoint(self, new_coord: tuple):
        dists = {}
        for key, val in self.rtcm_base_coords['RTCM32-Base'].items():
            dists[key] = haversine.haversine((val[0], val[1]), new_coord)
        self.current_mountpoint = min(dists.items(), key=lambda x: x[1])[0]

    def onRtcmPubTimerCallBack(self):
        try:
            
            raw_data, parsed_data = self.ntrip_queue.get()
            if protocol(raw_data) == RTCM3_PROTOCOL:
                self.rtcm_cnt += 1
                # self.get_logger().info("Message received: {}".format(parsed_data))
                rtcm_msg = Rtcm()
                rtcm_msg.message = raw_data
                rtcm_msg.header.frame_id = "rtcm_data"
                rtcm_msg.header.stamp = self.get_clock().now().to_msg()
                self.rtcm_pub.publish(rtcm_msg)
        except Exception as err:
            self.get_logger().error(f"Something went wrong in send thread {err}") 

        # Stop publishing rtcm data after 400 messages to check if rebase is needed or not
        if self.rtcm_cnt == REBASE_CHECK_COUNT:
            self.rtcm_cnt = 0
            self.rebase_check = True
            self.rtcmpub_timer.cancel()

            # Start subscribing to fix topic
            self.gnss_sub = self.create_subscription(NavSatFix, '/fix', self.onGnssSubCallBack, 1)
            
            

    def onGnssSubCallBack(self, msg):
        # self.gnc.set_ref_position(msg.latitude, msg.longitude, msg.altitude)  
        # self.get_logger().info(f"Current position: {msg.latitude}, {msg.longitude}, {msg.altitude}")
        rebase = False 
        if self.current_mountpoint == None:
            # First time connect to mountpoint
            self.reselect_mountpoint(new_coord=(msg.latitude, msg.longitude))
            self.get_logger().info(f"First time connect to mountpoint {self.current_mountpoint}")
            rebase = True

        elif self.rebase_check:
            # Check if rebase is needed or not
            self.get_logger().info(f"Checking if rebase is needed or not...")
            current_dist = haversine.haversine(self.rtcm_base_coords['RTCM32-Base'][self.current_mountpoint], (msg.latitude, msg.longitude))
            if current_dist > REBASE_DISTANCE_THRESHOLD:
                rebase = True
                self.reselect_mountpoint(new_coord=(msg.latitude, msg.longitude))

                self.gnc.stop()
                self.ntrip_queue.queue.clear()
                self.get_logger().info(f"Rebase to new mountpoint: {self.current_mountpoint}")
            else:
                self.get_logger().info(f"Current mountpoint is still valid: {self.current_mountpoint}")

                # Stop subscribing to fix topic
                self.destroy_subscription(self.gnss_sub)
                self.gnss_sub = None

                self.rtcmpub_timer.reset()
            

        # Try to connect to new mountpoint
        while rebase:
            try:
               # self.get_logger().info(f"Starting NTRIP client on {NTRIP_SERVER}:{NTRIP_PORT}...\n")
                self.get_logger().info(f"Connecting to mountpoint {self.current_mountpoint}...")
                self.streaming = self.gnc.run(
                    ipprot=IPPROT,
                    server=NTRIP_SERVER,
                    port=NTRIP_PORT,
                    flowinfo=FLOWINFO,
                    scopeid=SCOPEID,
                    mountpoint=self.current_mountpoint,
                    ntripuser=NTRIP_USER, 
                    ntrippassword=NTRIP_PASSWORD,
                    ggamode=GGAMODE,
                    ggainterval=GGAINT,
                    output=self.ntrip_queue,
                )
                self.get_logger().info(f"Connected to mountpoint {self.current_mountpoint}")

                # Stop subscribing to fix topic
                self.destroy_subscription(self.gnss_sub)
                self.gnss_sub = None

                # Start timer to publishing rtcm data
                self.rtcmpub_timer.reset() 

                rebase = False
                
            except Exception as err:
                self.get_logger().error(f"Something went wrong in run thread {err}")
                self.streaming = None
                self.ntrip_queue.queue.clear()
                continue


            self.rebase_check = False
            

def main(args=None):
    rclpy.init(args=args)
    rtcm_ntrip_pub = RtcmNtripPub()
    # executor = rclpy.executors.MultiThreadedExecutor()
    # executor.add_node(rtcm_ntrip_pub)
    # executor.spin()
    # executor.shutdown()
    rclpy.spin(rtcm_ntrip_pub)
    rtcm_ntrip_pub.gnc.stop()
    rtcm_ntrip_pub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()