<<<<<<< HEAD
=======
from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController
import rospy
>>>>>>> 30b170f... DBW with dbw_mkz_ros

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
<<<<<<< HEAD
    def __init__(self, *args, **kwargs):
        # TODO: Implement
        pass

    def control(self, *args, **kwargs):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        return 1., 0., 0.
=======
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit, accel_limit, wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):
        # TODO: Implement
	


        kp = 0.3 
        ki = 0.1
        kd = 0.0
        mn = 0.0
        mx = 0.2
        self.speed_pid = PID(kp, ki, kd, mn, mx)
        #self.accel_pid = PID(kp, ki)
        self.yaw_control = YawController(wheel_base, steer_ratio, 0.1, max_lat_accel, max_steer_angle) 

        tau = 0.5
        ts = 0.02
        self.lpf_vel = LowPassFilter(tau, ts)
        #self.lpf_accel = LowPassFilter(tau, ts)
        #self.lpf_fuel = LowPassFilter(tau, ts)	


        # parameter
        self.vehicle_mass = vehicle_mass + fuel_capacity * GAS_DENSITY
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius

        self.last_time = rospy.get_time()	

        self.control_period = 1/50.0  # 50Hz control rate 
        self.acker_wheelbase = wheel_base 
        self.steering_ratio = steer_ratio
        self.last_time = rospy.get_time()
        pass

    def control(self, current_linear_vel, dbw_enable, cmd_linear_vel, cmd_angular_vel):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        current_time = rospy.get_time()
        elasped_time = current_time - self.last_time
        self.last_time = current_time
        
	rospy.loginfo("current_linear_vel = %f, cmd_linear_vel = %f, cmd_angular_vel = %f)", current_linear_vel, cmd_linear_vel, cmd_angular_vel)
        if elasped_time > 10 * self.control_period:
            self.speed_pid.reset()
            #self.accel_pid.reset()
            return 0., 0., 0.

        current_linear_vel = self.lpf_vel.filt(current_linear_vel)
        vel_error = cmd_linear_vel - current_linear_vel
        if (abs(cmd_linear_vel) < 1.0 * ONE_MPH):
            self.speed_pid.reset()
        accel_cmd = self.speed_pid.step(vel_error, self.control_period)
        #if current_linear_vel < 0.01:
        #    accel_cmd = min(accel_cmd, -530 / self.vehicle_mass / self.wheel_radius)

        throttle_cmd = brake_cmd = steering_cmd = 0.0
        if dbw_enable:
            if accel_cmd > 0:
                # throttle_cmd = self.accel_pid(accel_cmd - self.lpf_accel.get(), control_period)
                throttle_cmd = accel_cmd
            else:
                #self.accel_pid.reset()
                throttle_cmd = 0.0
    		 	
            if accel_cmd < -self.brake_deadband:
                brake_cmd = -accel_cmd * self.vehicle_mass * self.wheel_radius
            else:
                brake_cmd = 0.0
    
            steering_cmd = self.yaw_control.get_steering(cmd_linear_vel, cmd_angular_vel, current_linear_vel) # + cfg.kp * (cmd_angular_vel - current_angular_vel)

        else:
            self.speed_pid.reset()
            #self.accel_pid.reset()
            return 0., 0., 0.	

        return throttle_cmd, brake_cmd, steering_cmd











>>>>>>> 30b170f... DBW with dbw_mkz_ros
