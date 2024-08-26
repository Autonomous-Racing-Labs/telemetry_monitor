import rclpy
from rclpy.node import Node

import itertools
import threading

import matplotlib.pyplot as plt
import numpy as np

import matplotlib.animation as animation

from visualization_msgs.msg import MarkerArray

from telemetry_monitor_interfaces.msg import Telemetry
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped


TOPIC_DEBUG_RACELINE = "/debug/raceline"
TOPIC_DEBUG_TELEMETRY = "/debug/telemetry"
TOPIC_LOCALIZATION_COVARIANCE = "/amcl_pose"

class telemetry_monitor(Node):
    def __init__(self):
        super().__init__('telemetry_monitor')
        self.__fig = None
        self.__animation = None
        self.__axTrack = None
        self.__axLateralDerivation = None
        self.__axSpeedProfile = None
        self.__axLocalizationUncertainty = None

        self.__plotLateralDerivation = None
        self.__plotSpeedProfileTarget = None
        self.__plotSpeedProfileActual = None
        self.__plotLocalizationUncertaintyX = None
        self.__plotLocalizationUncertaintyY = None
        self.__plotLocalizationUncertaintyYaw = None

        self.__raceline = { "xs": None, "ys": None, "cols": None }
        self.__actualLine = { "xs": [None for x in range(0, 1000)], "ys": [None for x in range(0, 1000)]}
        self.__lateralDerivation = []
        self.__speedProfile = {"target": [], "actual": []}
        self.__localizationUncertainty = {"x": [], "y": [], "yaw": []}

        #create subscribers
        self.__sub_raceline  = self.create_subscription(MarkerArray, TOPIC_DEBUG_RACELINE, self.cb_new_raceline, 1)
        self.__sub_telemetry = self.create_subscription(MarkerArray, TOPIC_DEBUG_TELEMETRY, self.cb_new_telemetry, 10)
        self.__sub_covariance = self.create_subscription(PoseWithCovarianceStamped, TOPIC_LOCALIZATION_COVARIANCE, self.cb_new_covariance, 10)

        self.setup_plots()


    def cb_new_covariance(self: "telemetry_monitor", msg: PoseWithCovarianceStamped):
        varx = msg.pose.covariance[0]
        vary = msg.pose.covariance[1]
        varyaw = msg.pose.covariance[35]

        self.__localizationUncertainty["x"].append(varx)
        self.__localizationUncertainty["y"].append(vary)
        self.__localizationUncertainty["yaw"].append(varyaw)

    def cb_new_telemetry(self: "telemetry_monitor", msg: Telemetry):
        #update driven line
        self.__actualLine["xs"][msg.next_waypoint_id] = msg.pos_x
        self.__actualLine["ys"][msg.next_waypoint_id] = msg.pos_y

        #update lateralDerivation
        self.__lateralDerivation.append(msg.lateral_derivation)

        #update speed profile
        self.__speedProfile["target"].append(msg.target_velocity)
        self.__speedProfile["actual"].append(msg.actual_velocity)


    def cb_new_raceline(self: "telemetry_monitor", msg: MarkerArray):
        xs = []
        ys = []
        cols = []

        for marker in msg.markers:
            xs.append(marker.pose.position.x)
            ys.append(marker.pose.position.y)
            cols.append( (marker.color.r, marker.color.g, marker.color.b, 1.0) )

        #move points such that origin is in (0,0)
        min_x = min(xs)
        min_y = min(ys)
        
        if min_x > 0:
            min_x *= -1
        if min_y > 0:
            min_y *= -1
        
        xs = [x-min_x for x in xs]
        ys = [y-min_y for y in ys]

        self.__raceline["xs"] = xs
        self.__raceline["ys"] = ys
        self.__raceline["cols"] = cols


    def setup_plots(self: "telemetry_monitor"):
        self.__fig, ((self.__axTrack, self.__axLateralDerivation), (self.__axSpeedProfile, self.__axLocalizationUncertainty)) = plt.subplots(2, 2)
        
        self.setup_track_plot()
        self.setup_lateral_derivation_plot()
        self.setup_speed_profile_plot()
        self.setup_localization_uncertainty_plot()

    def setup_track_plot(self: "telemetry_monitor"):
        self.__axTrack.scatter(self.__raceline["xs"], self.__raceline["ys"], c=self.__raceline["cols"])
        self.__axTrack.plot([x for x in self.__actualLine["xs"] if x is not None], 
                            [y for y in self.__actualLine["ys"] if y is not None], 
                            "b+", label="Driven Line")
        self.__axTrack.set_xlabel("X position [m]")
        self.__axTrack.set_ylabel("Y position [m]")
        self.__axTrack.set_title("Track")
        self.__axTrack.legend()


    def setup_lateral_derivation_plot(self: "telemetry_monitor"):
        self.__plotLateralDerivation, = self.__axLateralDerivation.plot(self.__lateralDerivation[-500:])
        self.__axLateralDerivation.set_xlabel("Epochs")
        self.__axLateralDerivation.set_ylabel("Derivation [m]")
        self.__axLateralDerivation.set_title("Lateral Derivation")
        pass

    def setup_speed_profile_plot(self: "telemetry_monitor"):
        self.__plotSpeedProfileTarget, = self.__axSpeedProfile.plot(self.__speedProfile["target"][-500:], label="Target", c="red")
        self.__plotSpeedProfileActual, = self.__axSpeedProfile.plot(self.__speedProfile["actual"][-500:], label="Actual", c="black")

        self.__axSpeedProfile.set_xlabel("Epochs")
        self.__axSpeedProfile.set_ylabel("Velocity [m/s]")
        self.__axSpeedProfile.set_title("Velocity Profile")
        self.__axSpeedProfile.legend()
        pass

    def setup_localization_uncertainty_plot(self: "telemetry_monitor"):
        self.__plotLocalizationUncertaintyX = self.__axLocalizationUncertainty.plot(self.__localizationUncertainty["x"][-500:], label="X", c="red")
        self.__plotLocalizationUncertaintyY = self.__axLocalizationUncertainty.plot(self.__localizationUncertainty["y"][-500:], label="Y", c="blue")
        self.__plotLocalizationUncertaintyYaw = self.__axLocalizationUncertainty.plot(self.__localizationUncertainty["yaw"][-500:], label="Yaw Angle", c="magenta")

        self.__axLocalizationUncertainty.set_xlabel("Epochs")
        self.__axLocalizationUncertainty.set_ylabel("Uncertainty [m] / [rad]")
        self.__axLocalizationUncertainty.set_title("Localization Uncertainty")
        self.__axLocalizationUncertainty.legend()
        pass

    #called whenever a frame of the plot is rendered
    def update_plots(self: "telemetry_monitor", arg):
        self.update_track_plot()
        self.update_lateral_derivation_plot()
        self.update_speed_profile_plot()
        self.setup_localization_uncertainty_plot()

    def update_track_plot(self: "telemetry_monitor"):
        #clear whole axis and start from the beginning. (lazy...)
        self.__axTrack.clear()
        self.setup_track_plot()

    def update_lateral_derivation_plot(self: "telemetry_monitor"):
        self.__plotLateralDerivation.set_data(self.__lateralDerivation[-500:])

    def update_speed_profile_plot(self: "telemetry_monitor"):
        self.__plotSpeedProfileTarget.set_data(self.__speedProfile["target"][-500:])
        self.__plotSpeedProfileActual.set_data(self.__speedProfile["actual"][-500:])

    def update_localization_uncertainty_plot(self: "telemetry_monitor"):
        self.__plotLocalizationUncertaintyX.set_data(self.__localizationUncertainty["x"][-500:])
        self.__plotLocalizationUncertaintyY.set_data(self.__localizationUncertainty["y"][-500:])
        self.__plotLocalizationUncertaintyYaw.set_data(self.__localizationUncertainty["yaw"][-500:])

    def show_plots(self: "telemetry_monitor"):
        self.__animation = animation.FuncAnimation(self.__fig, self.update_plots)
        plt.figure("Telemetry Monitor")
        plt.show(block=True)



    
def main(args=None):
    # launch telemetry_monitor node
    rclpy.init(args=args)
    telemetry_monitor_node = telemetry_monitor()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(telemetry_monitor_node)

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    telemetry_monitor_node.show_plots() #blocking call
    
    
    telemetry_monitor_node.destroy_node()
    rclpy.shutdown()
    executor_thread.join()


if __name__ == '__main__':
    main()
