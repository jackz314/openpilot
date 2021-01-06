#!/usr/bin/env python3
import time
import math
import atexit
import numpy as np
import threading
import argparse
import capnp

import cereal.messaging as messaging
from common.params import Params
from common.realtime import Ratekeeper, DT_DMON
from lib.can import can_function
from selfdrive.car.honda.values import CruiseButtons
from selfdrive.test.helpers import set_params_enabled

from joystick import LinuxVirtualJoystick

W, H = 1164, 874
REPEAT_COUNTER = 5
PRINT_DECIMATION = 100

from configparser import ConfigParser
cfg = ConfigParser()
cfg.read("./tweak.cfg")
STEER_RATIO = cfg.getfloat("steer", "STEER_RATIO", fallback=14)
STEER_SPEED_OFFSET = cfg.getfloat("steer", "STEER_SPEED_OFFSET", fallback=11)
BASE_MAX_STEER_ANGLE = cfg.getfloat("steer", "BASE_MAX_STEER_ANGLE", fallback=38.4)  # tweak to set base steer ratio/angle
STEER_SPEED_DENOM = cfg.getfloat("steer", "STEER_SPEED_DENOM", fallback=1.46)
print("tweak values: steer ratio:", STEER_RATIO, "steer speed offset:", STEER_SPEED_OFFSET,
      "max steer angle:", BASE_MAX_STEER_ANGLE, "steer speed denominator:", STEER_SPEED_DENOM)

pm = messaging.PubMaster(['frame', 'sensorEvents', 'can'])
sm = messaging.SubMaster(['carControl', 'controlsState'])


class VehicleState:
    def __init__(self):
        self.speed = 0
        self.angle = 0
        self.cruise_button = 0
        self.is_engaged = False
        self.left_blinker = False
        self.right_blinker = False


def steer_rate_limit(old, new):
    # if True: return new
    # Rate limiting to 0.5 degrees per step
    # print("limit:",old,new)
    # limit = 0.25 * (65536/360)
    limit = 1.5
    if new > old + limit:
        return old + limit
    elif new < old - limit:
        return old - limit
    else:
        return new


frame_id = 0
def cam_callback(image):
    # print(image.shape)
    global frame_id
    # img = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
    # img = np.reshape(img, (H, W, 4))
    # img = img[:, :, [0, 1, 2]].copy()

    dat = messaging.new_message('frame')
    dat.frame = {
        "frameId": frame_id,
        "image": image.tobytes(),
        "transform": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    }
    pm.send('frame', dat)
    frame_id += 1

import subprocess, time, os, sys
def cam_function():
    # pass
    cmd = ['cap/nvfbc_cap']
    p = subprocess.Popen(cmd, stdout=sys.stdout)

    def destroy():
        p.kill()
        print("killed capture service")

    atexit.register(destroy)
    p.wait()

    # context = zmq.Context()
    # s = context.socket(zmq.SUB)
    # s.setsockopt(zmq.SUBSCRIBE, b"")
    # s.connect("tcp://127.0.0.1:10245")

    # import cv2
    # test_img = cv2.imread("a.bmp")
    # while True:
    #     cam_callback(test_img)
    #     time.sleep(0.05)

    # from random import randrange
    # test_img = np.zeros((874,1164,3),np.uint8)
    # import mss
    # with mss.mss() as sct:
    #     mon = (0, 0, 3840, 2160)
    #     while True:
    #         # test_img[:] = (randrange(255),randrange(255),randrange(255))
    #         #  Wait for next request from client
    #         # m = s.recv()
    #         # print(f"Received: {m}")

    #         # buf = memoryview(m)
    #         # a = np.frombuffer(buf, dtype=np.uint8).reshape((H, W, 3))

    #         a = np.asarray(sct.grab(mon))
    #         a = cv2.resize(a, (1164,874))

    #         cam_callback(a)
    #         # time.sleep(0.01) # 60fps
import os
car = capnp.load(os.path.join("../../cereal", "car.capnp"))

def health_function():
    pm = messaging.PubMaster(['health'])
    while 1:
        dat = messaging.new_message('health')
        dat.valid = True
        dat.health = {
            'ignitionLine': True,
            'hwType': "blackPanda",
            'controlsAllowed': True,
            'safetyModel': car.CarParams.SafetyModel.hondaNidec,
        }
        pm.send('health', dat)
        time.sleep(0.5)

from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler

class ConfigModifiedHandler(FileSystemEventHandler):
    def __init__(self, path, file_name, callback):
        self.path = path
        self.file_name = file_name
        self.callback = callback

        self.observer = Observer()
        self.observer.schedule(self, path, recursive=False)
        self.observer.start()

    def join(self):
        self.observer.join()

    def __del__(self):
        self.observer.stop()

    def on_modified(self, event):
        if event.src_path.endswith(self.file_name) and not event.is_directory:
            self.callback()

def config_change_callback():
    cfg.read("./tweak.cfg")
    global STEER_RATIO, STEER_SPEED_OFFSET, BASE_MAX_STEER_ANGLE, STEER_SPEED_DENOM
    STEER_RATIO = cfg.getfloat("steer", "STEER_RATIO", fallback=14)
    STEER_SPEED_OFFSET = cfg.getfloat("steer", "STEER_SPEED_OFFSET", fallback=11)
    BASE_MAX_STEER_ANGLE = cfg.getfloat("steer", "BASE_MAX_STEER_ANGLE", fallback=38.4)
    STEER_SPEED_DENOM = cfg.getfloat("steer", "STEER_SPEED_DENOM", fallback=1.46)
    print("new tweak values: steer ratio:", STEER_RATIO, "steer speed offset:", STEER_SPEED_OFFSET,
          "max steer angle:", BASE_MAX_STEER_ANGLE, "steer speed denominator:", STEER_SPEED_DENOM)

def config_watcher():
    handler = ConfigModifiedHandler("./", "tweak.cfg", config_change_callback)
    handler.join()

from scssdk import scssdkclient

def imu_callback(s: scssdkclient):
    dat = messaging.new_message('sensorEvents', 1)
    dat.sensorEvents[0].sensor = 4
    dat.sensorEvents[0].type = 0x10
    dat.sensorEvents[0].init('acceleration')
    dat.sensorEvents[0].acceleration.v = [s.accelerationX, s.accelerationY, s.accelerationZ]
    # copied these numbers from locationd
    # dat.sensorEvents[1].sensor = 5
    # dat.sensorEvents[1].type = 0x10
    # dat.sensorEvents[1].init('gyroUncalibrated')
    # dat.sensorEvents[1].gyroUncalibrated.v = [0, 0, 0]
    pm.send('sensorEvents', dat)

def imu_function():
    if True: return
    s = scssdkclient()
    while True:
        s.update()
        imu_callback(s)
        time.sleep(0.01)

def fake_gps():
    # TODO: read GPS from truck sim, also speed limit
    pm = messaging.PubMaster(['gpsLocationExternal'])
    while 1:
        dat = messaging.new_message('gpsLocationExternal')
        pm.send('gpsLocationExternal', dat)
        time.sleep(0.01)


def fake_driver_monitoring():
    pm = messaging.PubMaster(['driverState', 'dMonitoringState'])
    while 1:
        # dmonitoringmodeld output
        dat = messaging.new_message('driverState')
        dat.driverState.faceProb = 1.0
        pm.send('driverState', dat)

        # dmonitoringd output
        dat = messaging.new_message('dMonitoringState')
        dat.dMonitoringState = {
            "faceDetected": True,
            "isDistracted": False,
            "awarenessStatus": 1.,
            "isRHD": False,
        }
        pm.send('dMonitoringState', dat)

        time.sleep(DT_DMON)  # 0.1


def can_function_runner(vs):
    i = 0
    while 1:
        can_function(pm, vs.speed, vs.angle, i, vs.cruise_button, vs.is_engaged, vs.left_blinker, vs.right_blinker)
        time.sleep(0.01)
        i += 1

def go(q):
    # camera.listen(cam_callback)

    # imu.listen(imu_callback)

    def destroy():
        print("clean exit")

        print("done")

    atexit.register(destroy)

    vehicle_state = VehicleState()

    # launch fake car threads
    # threading.Thread(target=cam_function).start()
    # time.sleep(1)
    # threading.Thread(target=imu_function).start()
    threading.Thread(target=config_watcher).start()
    threading.Thread(target=health_function).start()
    threading.Thread(target=fake_driver_monitoring).start()
    threading.Thread(target=fake_gps).start()
    threading.Thread(target=can_function_runner, args=(vehicle_state,)).start()

    # can loop
    rk = Ratekeeper(100, print_delay_threshold=0.05)

    # init
    throttle_ease_out_counter = REPEAT_COUNTER
    brake_ease_out_counter = REPEAT_COUNTER
    steer_ease_out_counter = REPEAT_COUNTER

    # vc = carla.VehicleControl(throttle=0, steer=0, brake=0, reverse=False)

    is_openpilot_engaged = False
    throttle_out = steer_out = brake_out = 0
    throttle_op = steer_op = brake_op = 0
    throttle_manual = steer_manual = brake_manual = 0

    old_steer = old_brake = old_throttle = 0
    throttle_manual_multiplier = 0.7  # keyboard signal is always 1
    brake_manual_multiplier = 0.7  # keyboard signal is always 1
    steer_manual_multiplier = 45 * STEER_RATIO  # keyboard signal is always 1

    joy = LinuxVirtualJoystick()
    s = scssdkclient()

    while 1:
        # 1. Read the throttle, steer and brake from op or manual controls
        # 2. Set instructions in Carla
        # 3. Send current carstate to op via can

        cruise_button = 0
        throttle_out = steer_out = brake_out = 0
        throttle_op = steer_op = brake_op = 0
        throttle_manual = steer_manual = brake_manual = 0

        # --------------Step 1-------------------------------
        if not q.empty():
            message = q.get()
            m = message.split('_')
            if m[0] == "cruise":
                if m[1] == "down":
                    cruise_button = CruiseButtons.DECEL_SET
                    is_openpilot_engaged = True
                if m[1] == "up":
                    cruise_button = CruiseButtons.RES_ACCEL
                    is_openpilot_engaged = True
                if m[1] == "cancel":
                    cruise_button = CruiseButtons.CANCEL
                    is_openpilot_engaged = False

            throttle_out = throttle_manual * throttle_manual_multiplier
            steer_out = steer_manual * steer_manual_multiplier
            brake_out = brake_manual * brake_manual_multiplier

            # steer_out = steer_out
            # steer_out = steer_rate_limit(old_steer, steer_out)
            old_steer = steer_out
            old_throttle = throttle_out
            old_brake = brake_out

            # print('message',old_throttle, old_steer, old_brake)

        if is_openpilot_engaged:
            sm.update(0)
            throttle_op = sm['carControl'].actuators.gas  # [0,1]
            brake_op = sm['carControl'].actuators.brake  # [0,1]
            steer_op = sm['controlsState'].angleSteersDes  # degrees [-180,180]

            throttle_out = throttle_op
            steer_out = steer_op
            brake_out = brake_op

            steer_out = steer_rate_limit(old_steer, steer_out)
            old_steer = steer_out

        # OP Exit conditions
        # if throttle_out > 0.3:
        #   cruise_button = CruiseButtons.CANCEL
        #   is_openpilot_engaged = False
        # if brake_out > 0.3:
        #   cruise_button = CruiseButtons.CANCEL
        #   is_openpilot_engaged = False
        # if steer_out > 0.3:
        #   cruise_button = CruiseButtons.CANCEL
        #   is_openpilot_engaged = False

        else:
            if throttle_out == 0 and old_throttle > 0:
                if throttle_ease_out_counter > 0:
                    throttle_out = old_throttle
                    throttle_ease_out_counter += -1
                else:
                    throttle_ease_out_counter = REPEAT_COUNTER
                    old_throttle = 0

            if brake_out == 0 and old_brake > 0:
                if brake_ease_out_counter > 0:
                    brake_out = old_brake
                    brake_ease_out_counter += -1
                else:
                    brake_ease_out_counter = REPEAT_COUNTER
                    old_brake = 0

            if steer_out == 0 and old_steer != 0:
                if steer_ease_out_counter > 0:
                    steer_out = old_steer
                    steer_ease_out_counter += -1
                else:
                    steer_ease_out_counter = REPEAT_COUNTER
                    old_steer = 0

        # --------------Step 2-------------------------------

        # todo send commands back to truck sim

        # steer_truck = steer_out
        # steer_out = old_steer = steer_truck

        s.update()
        imu_callback(s)

        max_steer_angle = BASE_MAX_STEER_ANGLE + max(0, s.speed - STEER_SPEED_OFFSET) / STEER_SPEED_DENOM # decrease the constant to decrease change of steer ration per change of speed
        # max_steer_angle = 59 # alternatively, use a constant ratio

        steer_truck = steer_out / (max_steer_angle * STEER_RATIO * 1)

        steer_truck = np.clip(steer_truck, -1,1)
        steer_out = steer_truck * (max_steer_angle * STEER_RATIO * 1)
        old_steer = steer_truck * (max_steer_angle * STEER_RATIO * 1)

        throttle = throttle_out * 65536 / 0.5
        steer = steer_truck * 32768
        brake = brake_out * 65536 / 0.9
        # brake = 0
        # print(throttle, steer, brake)
        joy.emit(steer, throttle, brake)
        # --------------Step 3-------------------------------
        # vel = vehicle.get_velocity()
        # speed = math.sqrt(vel.x**2 + vel.y**2 + vel.z**2) # in m/s

        vehicle_state.speed = 0 if s.paused else max(s.speed,0)
        vehicle_state.angle = steer_out
        # vehicle_state.angle = s.gameSteer # [-1,1]
        vehicle_state.cruise_button = cruise_button
        vehicle_state.is_engaged = is_openpilot_engaged
        if not vehicle_state.left_blinker and s.blinkerLeftActive: print("left blinker on")
        if not vehicle_state.right_blinker and s.blinkerRightActive: print("right blinker on")
        vehicle_state.left_blinker = s.blinkerLeftActive
        vehicle_state.right_blinker = s.blinkerRightActive
        # if s.userSteer: print(s.userSteer)
        if rk.frame % PRINT_DECIMATION == 0:
            # print("frame: ", "engaged:", is_openpilot_engaged, "; throttle: ", round(vc.throttle, 3), "; steer(c/deg): ", round(vc.steer, 3), round(steer_out, 3), "; brake: ", round(vc.brake, 3))
            print("frame: ", "engaged:", is_openpilot_engaged, "; steering:", round(steer_out, 5), "game steer:", s.gameSteer, "throttle:",
                  round(throttle_out, 5), "brake:", round(brake_out, 5))

        rk.keep_time()


if __name__ == "__main__":

    # make sure params are in a good state
    params = Params()
    params.clear_all()
    set_params_enabled()
    params.delete("Offroad_ConnectivityNeeded")
    # uncomment to skip calibration
    # params.put("CalibrationParams", '{"calib_radians": [0,0,0], "valid_blocks": 20}')

    from multiprocessing import Process, Queue

    q = Queue()
    p = Process(target=go, args=(q,))
    p.daemon = True
    p.start()

    fp = Process(target=cam_function)
    fp.daemon = True
    fp.start()

    # start input poll for keyboard
    # from lib.keyboard_ctrl import keyboard_poll_thread
    # keyboard_poll_thread(q)

    from pynput import keyboard


    def on_press(key):
        try:
            # print('alphanumeric key {0} pressed'.format(key.char))
            if key.char == 'c': op_plus() # press 'c' to increase OP speed
            elif key.char == 'z': op_minus() # press 'z' to decrease OP speed
            elif key.char == 'v': op_cancel() # press 'v' to cancel OP
        except AttributeError:
            pass
            # print('special key {0} pressed'.format(key))


    def op_plus():
        print("OP++")
        q.put(str("cruise_up"))

    def op_minus():
        print("OP--")
        q.put(str("cruise_down"))

    def op_cancel():
        print("OP Cancel")
        q.put(str("cruise_cancel"))

    # Collect events until released
    with keyboard.Listener(
        on_press=on_press,
        on_release=None) as listener:
        listener.join()

    # time.sleep(30)
    # q.put(str("cruise_up"))

    p.join()
    fp.join()
