# Numpy imports
from numpy import interp, array, deg2rad, rad2deg

# ROS imports
import rclpy
from rclpy.qos import qos_profile_sensor_data

# Message imports
from px4_msgs.msg import InputRc

# Morphing Lander imports
from phoenix.mpc.TiltControllerBase import TiltControllerBase
from phoenix.mpc.roboclaw_3 import Roboclaw
from phoenix.mpc.parameters import params_

import time
from collections import deque

min                    = params_['min']
max                    = params_['max']
dead                   = params_['dead']
Ts                     = params_.get('Ts_tilt_controller')
tilt_roboclaw_address  = params_.get('tilt_roboclaw_address')
tilt_channel1           = params_.get('tilt_channel1')
tilt_channel2           = params_.get('tilt_channel2')
encoder_channel        = params_.get('encoder_channel')
offboard_channel       = params_.get('offboard_channel')

tilt_switch_channel    = params_.get('tilt_switch_channel')   # = 8 # min disable tilt, max enable tilt
# drive_switch_channel   = params_.get('drive_switch_channel')  # = 4 # 1514, middle(dead) -> drive mod
kill_switch_channel    = params_.get('kill_switch_channel')   # = 5 # kill Flymod also kill drive and tilt



class TiltHardware(TiltControllerBase):
    def __init__(self):
        super().__init__()

        # Subscriptions
        self.rc_subscription = self.create_subscription(
            InputRc,
            '/fmu/out/input_rc',
            self.rc_listener_callback,
            qos_profile_sensor_data)
        self.rc_subscription        # rc subscription

        # Configure RC inputs
        self.min = min 
        self.max = max 
        self.dead = dead 

        # Initialize LS_in
        self.LS_in1 = dead
        self.LS_in2 = dead

        # Initialize switch_trigger
        self.tilt_switch_trigger = min   # min -> disarmd
        self.kill_switch_trigger = max  # kill system

        # Manual vs automatic control
        self.manual = True

        # Initialize roboclaw at given address
        self.address = 0x80
        self.rc = Roboclaw(tilt_roboclaw_address,115200)
        self.rc.Open()

        # Set encoder count to zero (always start robot in drive configuration)
        # M1 left, M2 right
        self.rc.SetEncM1(self.address,0)
        self.rc.SetEncM2(self.address,0)
        self.reset_encoder = 0

        # Set pin functions for motor 2 (M2) to go to zero when it reaches home (limit switch)
        # seems like here activate S4 and S5, S4 for channal 1, S5 for 2
        # this line not sure whether it is working, need to use Roboclaw wizard to set pin function(limit both)
        self.rc.SetPinFunctions(self.address,0x00,0x62,0x62) 

        # Encoder data for tilt angle publishing (manually converting between data which was collected for encoder 44 from pololu to encoder 45. I should recalibrate)
        self.encoder_data = array([0,3696,5551,7647,8886,10118,11062,11982,12846,13957,14885,15629,16549,17037,17749,19029,19645,20989,21453,22357,22989,23517,24013,24605,25437,25973,26325])
        self.angle_data = deg2rad(array([90,86.6,84.0,78.6,74.2,70.5,66.5,63.5,59.7,55.3,51.3,48.0,44.2,41.8,38.4,32.8,30.0,24.2,22.0,18.0,14.8,12.9,10.8,8.2,4.7,2.8,0.8]))

        # limit tilt
        self.limit_tilt = deg2rad(1.0)

        self.speed_fillter_count = 0
        self.speed_fillter_size = 100

        self.speed_fillter_1 = deque(maxlen=self.speed_fillter_size)
        self.speed_fillter_2 =  deque(maxlen=self.speed_fillter_size)

        self.speed_timer =  deque(maxlen=self.speed_fillter_size)

    def rc_listener_callback(self, msg):
        # https://futabausa.com/wp-content/uploads/2018/09/18SZ.pdf
        self.LS_in1 = msg.values[tilt_channel1] # corresponds to LS trim selector on futaba T18SZ that I configured in the function menu
        self.LS_in2 = msg.values[tilt_channel2] # corresponds to LS trim selector on futaba T18SZ that I configured in the function menu

        self.tilt_switch_trigger = msg.values[tilt_switch_channel]   # min -> disarmd
        self.kill_switch_trigger = msg.values[kill_switch_channel]
        # reset encoder 
        self.reset_encoder = msg.values[encoder_channel]

        # set manual or automatic control of tilt angle
        if msg.values[offboard_channel] == self.max:
            self.manual = False
        else:
            self.manual = True

    def normalize(self,LS_in):
        return (float(LS_in)-float(self.dead))/(self.max-self.dead)

    def map_speed(self,speed_normalized):
        return int(127*speed_normalized)

    def stop(self):
        self.rc.ForwardM2(self.address,self.map_speed(0.0))

    def spin_motor(self, tilt_speed, motor_select):
        # takes in a tilt_speed between -1 and 1 writes the pwm signal to the roboclaw 

        # limit speed when reaching the limit tilt angle
        # if (self.tilt_angle < self.limit_tilt):
        #     if (tilt_speed < 0) : tilt_speed = 0.0
        #     else: pass  

        motor_speed = self.map_speed(abs(tilt_speed))
        if (tilt_speed < 0): # go up
            if motor_speed >= 127: 
                motor_speed = 126 # weird bug not sure why this is needed

            if motor_select == 1:
                self.rc.ForwardM1(self.address, motor_speed)
            elif motor_select ==2:
                self.rc.ForwardM2(self.address, motor_speed)

        else:
            if motor_speed >= 127:
                motor_speed = 126

            if motor_select == 1:
                self.rc.BackwardM1(self.address, motor_speed)
            elif motor_select ==2:
                self.rc.BackwardM2(self.address, motor_speed)


    def on_shutdown(self):
        self.stop()
        self.rc._port.close()
        self.get_logger().info("port closed !")

    def reset_encoder_trigger(self):
        if (self.reset_encoder == self.max):
            self.rc.SetEncM1(self.address,0)
            self.rc.SetEncM2(self.address,0)

    def get_current_tilt_angle(self):
        # compute and print current tilt angle
        enc_count1 = self.rc.ReadEncM1(self.address)
        enc_count2 = self.rc.ReadEncM2(self.address)

        self.tilt_angle1 =  float(interp(enc_count1[1],self.encoder_data,self.angle_data))
        self.tilt_angle2 =  float(interp(enc_count2[1],self.encoder_data,self.angle_data))

        # print(f"tilt angle is: {rad2deg(self.tilt_angle1),rad2deg(self.tilt_angle2) }")
        print(f"encoder count is: {enc_count1, enc_count2}")

        # self.speed_fillter_count +=1
        # self.speed_fillter_1.append(enc_count1)
        # self.speed_fillter_2.append(enc_count2)
        # self.speed_timer.append(time.time())

        # if(self.speed_fillter_count > self.speed_fillter_size ):
            
        #     speed_1 = float(self.speed_fillter_1[-1][1] - self.speed_fillter_1[0][1]) / float(self.speed_timer[-1] - self.speed_timer[0])
        #     speed_2 = float(self.speed_fillter_2[-1][1] - self.speed_fillter_2[0][1]) / float(self.speed_timer[-1] - self.speed_timer[0])
        #     print(f"speed count is: {speed_1, speed_2}")
        #     print(f"speed count is: {speed_1 - speed_2}")

        

        return self.tilt_angle1, self.tilt_angle1

    def update(self):
        if(self.tilt_switch_trigger == max and self.kill_switch_trigger == min):

            if (self.manual):
                # manual control of tilt angle
                # self.LS_in1 = self.LS_in1 + 1
                # if self.LS_in1 > max-200:
                #     self.LS_in1 = max-200

                # self.LS_in2 = self.LS_in2 + 1
                # if self.LS_in2 > max-200:
                #     self.LS_in2 = max-200
                # self.LS_in1 = dead - 100
                # self.LS_in2 = dead - 100

                # 1 is left motor 2 is right
                u1 = self.normalize(self.LS_in1)
                u2 = self.normalize(self.LS_in2)
                print(f"tilt u is: {u1}")

                # u = 0.5
                self.spin_motor(u1,1)
                self.spin_motor(u2*0.95,2)

            else:

                self.spin_motor(self.tilt_vel1,1)

        else:
            self.spin_motor(0,1)
            self.spin_motor(0,2)
            print("disable tilting")

def main(args=None):
    rclpy.init(args=args)
    tilt_controller = TiltHardware()
    tilt_controller.get_logger().info("Starting TiltHardware node...")
    rclpy.spin(tilt_controller)
    tilt_controller.on_shutdown()  # do any custom cleanup
    tilt_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()