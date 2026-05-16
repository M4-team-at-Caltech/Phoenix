import rclpy
from rclpy.qos import qos_profile_sensor_data

from px4_msgs.msg import InputRc

from phoenix.mpc.DriveControllerBase import DriveControllerBase
from phoenix.cubemars_driver import MotorManager
from phoenix.mpc.parameters import params_

# RC parameters
min                  = params_.get('min')
max                  = params_.get('max')
dead                 = params_.get('dead')

pitch_channel        = params_.get('pitch_channel')
roll_channel         = params_.get('roll_channel')
offboard_channel     = params_.get('offboard_channel')
drive_switch_channel = params_.get('drive_switch_channel')
kill_switch_channel  = params_.get('kill_switch_channel')

# CubeMars CAN parameters
can_channel          = params_.get('can_channel')
left_can_id          = params_.get('left_can_id')
right_can_id         = params_.get('right_can_id')
erpm_max             = params_.get('erpm_max')
allow_max_rate       = params_.get('allow_max_rate')


class DriveControllerHardware(DriveControllerBase):
    def __init__(self):
        super().__init__()

        self.subscription = self.create_subscription(
            InputRc,
            '/fmu/out/input_rc',
            self.rc_listener_callback,
            qos_profile_sensor_data)
        self.subscription  # prevent unused variable warning

        # initialize drive speed and turn speed
        self.drive_speed_in = dead
        self.turn_speed_in  = dead

        # initialize drive switch and kill switch
        self.drive_switch_trigger = min   # min -> disarmed
        self.kill_switch_trigger  = max   # kill system

        # Manual vs automatic control
        self.manual = True

        # Compute actual ERPM ceiling from hardware limit and allowed rate
        self.actual_max_erpm = int(erpm_max * allow_max_rate)
        self.get_logger().info(
            f"CubeMars drive: erpm_max={erpm_max}, allow_max_rate={allow_max_rate}, "
            f"actual_max_erpm={self.actual_max_erpm}"
        )

        # Open CAN bus and initialize both drive motors
        self.motor_manager = MotorManager(
            channel=can_channel,
            motor_ids=(left_can_id, right_can_id)
        )

    def _apply_deadzone(self, value, deadzone=10):
        if abs(value - dead) <= deadzone:
            return dead
        return value

    def rc_listener_callback(self, msg):
        self.drive_speed_in       = self._apply_deadzone(msg.values[pitch_channel])
        self.turn_speed_in        = self._apply_deadzone(msg.values[roll_channel])
        self.drive_switch_trigger = msg.values[drive_switch_channel]
        self.kill_switch_trigger  = msg.values[kill_switch_channel]

        if msg.values[offboard_channel] == max:
            self.manual = False
        else:
            self.manual = True

    def normalize(self, rc_value):
        return (float(rc_value) - float(dead)) / float(max - dead)

    def map_speed(self, speed_normalized, vel_deadzone=0.01):
        if abs(speed_normalized) < vel_deadzone:
            return 0
        return int(self.actual_max_erpm * speed_normalized)

    def move_right_wheel(self, erpm):
        self.motor_manager.set_velocity(right_can_id, -erpm)

    def move_left_wheel(self, erpm):
        # Left motor is physically reversed
        self.motor_manager.set_velocity(left_can_id, erpm)

    def on_shutdown(self):
        self.motor_manager.stop_all()
        self.motor_manager.disable_all()
        self.motor_manager.close()
        self.get_logger().info("CAN bus closed.")

    def update(self):
        if self.drive_switch_trigger == dead and self.kill_switch_trigger == min:
            if self.manual:
                lin_vel = -1.0 * self.map_speed(self.normalize(self.drive_speed_in))
                ang_vel = -0.5 * self.map_speed(self.normalize(self.turn_speed_in))

                self.get_logger().info(
                    f"lin_vel, ang_vel: ({lin_vel}, {ang_vel})"
                )

                self.motor_manager.set_all_velocity({
                    right_can_id:  lin_vel + ang_vel,
                    left_can_id:  -(lin_vel - ang_vel),  # left reversed
                })
            else:
                # automatic control: drive_speed / turn_speed come from DriveControllerBase
                lin_vel = -self.map_speed(self.drive_speed)
                ang_vel = -self.map_speed(self.turn_speed)

                self.motor_manager.set_all_velocity({
                    right_can_id:  -lin_vel + ang_vel,
                    left_can_id:  (lin_vel - ang_vel),  # left reversed
                })
        else:
            self.motor_manager.set_all_velocity({
                right_can_id: 0,
                left_can_id:  0,
            })
            print("pause driving")


def main(args=None):
    rclpy.init(args=args)
    drive_controller = DriveControllerHardware()
    drive_controller.get_logger().info("Starting DriveControllerHardware node...")
    rclpy.spin(drive_controller)
    drive_controller.on_shutdown()
    drive_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
