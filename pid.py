#!/usr/bin/env python3 
# import python libraries 
import time 
 
# import rcpy library 
# This automatically initializes the robotics cape 
import rcpy 
import rcpy.mpu9250 as mpu9250 
import rcpy.servo as servo 
import rcpy.clock as clock 
 
rcpy.set_state(rcpy.RUNNING) 
mpu9250.initialize(enable_dmp=True, dmp_sample_rate=100, enable_fusion=True, enable_magnetometer=True) 
 
srvo = servo.Servo(1) 
clck = clock.Clock(srvo, 0.02) 
servo.enable() 
clck.start() 
srvo.set(-1.0) 
 
# countdown 
for i in range(0, 10): 
    print(i) 
    time.sleep(1.0) 
 
# settings 
kp = 0.0035  # setting for P 
ki = 0.003   # setting for I 
kd = 0.0035  # setting for D 
 
# initial readings 
i = 0.0 
data = mpu9250.read() 
t0 = time.time() 
e0 = (data['tb'][0] / (2.0 * 3.1416)) * 360.0 
 
while True: 
    data = mpu9250.read()                           # read IMU 
    t1 = time.time() 
    dt = t1 - t0                                    # change in time since last reading 
    e1 = (data['tb'][0] / (2.0 * 3.1416)) * 360.0   # pitch angle 
 
    p = e1                                          # p is error amount 
    i = i + e1 * dt                                 # i is cumulative error over time 
    d = (e1 - e0) / dt                              # d is change in error over time 
 
    pid = kp * p + ki * i + kd * d                  # math for PID 
    pid = max(pid, -1.0) 
    pid = min(pid, 1.0) 
 
    throttle = -1.0 + (pid * 2.0)                   # throttle ESC range is -1.0 to 1.0 
    throttle = max(throttle, -1.0) 
    throttle = min(throttle, 1.0) 
 
    srvo.set(throttle) 
 
    print(p, i, d, throttle) 
    time.sleep(0.05) 
    t0 = t1 
    e0 = e1
    
