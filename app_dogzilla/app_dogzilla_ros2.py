#!/usr/bin/env python3
# coding=utf-8
# app_dogzilla_ros2.py
# Variante ROS 2 de app_dogzilla.py.
# Même protocole TCP smartphone, même flux MJPEG Flask —
# mais toutes les commandes hardware transitent par les topics ROS 2
# (yahboom_ctrl doit tourner en parallèle).
#
# Usage : python3 app_dogzilla_ros2.py [debug]

from flask import Flask, render_template, Response
import socket
import os
import time
import threading
import cv2 as cv
import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Int32, String, Bool, Float32

from camera_dogzilla import Dogzilla_Camera
from joystick_dogzilla import Dogzilla_Joystick
from oled_dogzilla import Dogzilla_OLED

from gevent import pywsgi


g_debug = False
if len(sys.argv) > 1 and sys.argv[1] == 'debug':
    g_debug = True
print('debug=', g_debug)


# ── ROS 2 bridge ─────────────────────────────────────────────────────────────

class DogzillaROS2(Node):
    """Remplaçant drop-in de DOGZILLA() — publie sur les topics ROS 2
    consommés par yahboom_ctrl au lieu d'écrire sur le port série."""

    RATE = 40  # doit correspondre à yahboom_ctrl.RATE

    def __init__(self):
        super().__init__('app_dogzilla_ros2')

        mk = lambda t, tp: self.create_publisher(t, tp, 10)
        self._pub_vel   = mk(Twist,   '/cmd_vel')
        self._pub_act   = mk(Int32,   '/dogzilla/action')
        self._pub_pace  = mk(String,  '/dogzilla/pace')
        self._pub_trans = mk(Vector3, '/dogzilla/translation')
        self._pub_att   = mk(Vector3, '/dogzilla/attitude')
        self._pub_imu   = mk(Bool,    '/dogzilla/imu')
        self._pub_perf  = mk(Int32,   '/dogzilla/perform')

        self._battery = 0.0
        self.create_subscription(Float32, '/battery_voltage',
                                 lambda m: setattr(self, '_battery', m.data), 10)

        # état local translation/attitude pour animations
        self._tx = self._ty = 0.0
        self._tz = 105.0
        self._roll = self._pitch = self._yaw = 0.0

    # ── mouvement ─────────────────────────────────────────────────────────────

    def _vel(self, vx=0.0, vy=0.0, wz=0.0):
        msg = Twist()
        msg.linear.x  = float(vx)
        msg.linear.y  = float(vy)
        msg.angular.z = float(wz)
        self._pub_vel.publish(msg)

    def stop(self):          self._vel()
    def move_x(self, s):    self._vel(vx= s / self.RATE)
    def move_y(self, s):    self._vel(vy= s / self.RATE)
    def turn(self, s):      self._vel(wz= s / self.RATE)
    def forward(self, s):   self._vel(vx= s / self.RATE)
    def back(self, s):      self._vel(vx=-s / self.RATE)
    def left(self, s):      self._vel(vy= s / self.RATE)
    def right(self, s):     self._vel(vy=-s / self.RATE)
    def turnleft(self, s):  self._vel(wz= s / self.RATE)
    def turnright(self, s): self._vel(wz=-s / self.RATE)

    # ── actions / pace ────────────────────────────────────────────────────────

    def action(self, action_id):
        msg = Int32(); msg.data = int(action_id)
        self._pub_act.publish(msg)

    def pace(self, mode):
        msg = String(); msg.data = str(mode)
        self._pub_pace.publish(msg)

    def perform(self, mode):
        msg = Int32(); msg.data = int(mode)
        self._pub_perf.publish(msg)

    def imu(self, state):
        msg = Bool(); msg.data = bool(state)
        self._pub_imu.publish(msg)

    # ── posture ───────────────────────────────────────────────────────────────

    def translation(self, axis, val):
        if axis == 'x':   self._tx = float(val)
        elif axis == 'y': self._ty = float(val)
        elif axis == 'z': self._tz = float(val)
        msg = Vector3()
        msg.x, msg.y, msg.z = self._tx, self._ty, self._tz
        self._pub_trans.publish(msg)

    def attitude(self, axis, val):
        if isinstance(axis, list):
            for a, v in zip(axis, val):
                if a == 'r':   self._roll  = float(v)
                elif a == 'p': self._pitch = float(v)
                elif a == 'y': self._yaw   = float(v)
        else:
            if axis == 'r':   self._roll  = float(val)
            elif axis == 'p': self._pitch = float(val)
            elif axis == 'y': self._yaw   = float(val)
        msg = Vector3()
        msg.x, msg.y, msg.z = self._roll, self._pitch, self._yaw
        self._pub_att.publish(msg)

    # ── télémétrie ────────────────────────────────────────────────────────────

    def read_battery(self):
        return self._battery

    def motor_speed(self, speed):
        pass  # non exposé via topics

    def read_motor(self):
        return [0] * 12  # TODO : souscrire à /joint_states

    def motor(self, ids, vals):
        pass  # non exposé via topics

    def leg(self, leg_id, vals):
        pass  # non exposé via topics

    def calibration(self, state):
        pass  # volontairement désactivé côté ROS 2


# ── init ROS 2 dans un thread dédié ──────────────────────────────────────────

rclpy.init()
g_dog = DogzillaROS2()
_ros_thread = threading.Thread(target=rclpy.spin, args=(g_dog,), daemon=True)
_ros_thread.start()


# ── reste identique à app_dogzilla.py ────────────────────────────────────────

g_camera = Dogzilla_Camera(debug=g_debug)
g_oled   = Dogzilla_OLED(g_dog, debug=g_debug)

g_tcp_ip = 'x.x.x.x'
g_init   = False
g_mode   = 'Home'

app = Flask(__name__)

g_step_control       = 50
g_pace_freq          = 2
g_motor_speed        = [0, 0, 0, 0]
g_car_stabilize_state = 0

STEP_SCALE_X = 0.2
STEP_SCALE_Y = 0.2
STEP_SCALE_Z = 0.7

g_height           = 108
g_shoulder         = 0
g_action_continuous = 0
g_press_up         = 0
g_motor_id         = 1
g_tcp_except_count = 0


def my_map(x, in_min, in_max, out_min, out_max):
    return (out_max - out_min) * (x - in_min) / (in_max - in_min) + out_min


def hex2int(str_hex, HEX=True):
    num = int(str_hex, 16)
    if HEX:
        return num
    return num - 256 if num > 127 else num


def int2hex(v):
    if v < -128 or v > 255:
        v = 0
    return v + 256 if v < 0 else v


def get_ip_address():
    ip = os.popen("/sbin/ifconfig eth0 | grep 'inet' | awk '{print $2}'").read()
    ip = ip[:ip.find('\n')]
    if not ip or len(ip) > 15:
        ip = os.popen("/sbin/ifconfig wlan0 | grep 'inet' | awk '{print $2}'").read()
        ip = ip[:ip.find('\n')]
    if not ip or len(ip) > 15:
        ip = 'x.x.x.x'
    return ip


def dogzilla_reset():
    global g_height, g_shoulder, g_action_continuous, g_press_up, g_pace_freq
    g_height = 108; g_shoulder = 0
    g_action_continuous = 0; g_press_up = 0; g_pace_freq = 2
    g_dog.action(0xff)


def dogzilla_leg_reset():
    for leg in range(1, 5):
        g_dog.leg(leg, [0, 0, 108])
        time.sleep(.005)


# ── retours TCP vers le smartphone ───────────────────────────────────────────

def return_battery_voltage():
    T_TYPE, T_FUNC, T_LEN = 0x01, 0x02, 0x06
    vol = int(g_dog.read_battery())
    chk = (T_TYPE + T_FUNC + T_LEN + vol) % 256
    _tcp_send("$%02x%02x%02x%02x%02x%02x#" % (T_TYPE, T_FUNC, T_LEN, vol, 0, chk))
    if g_debug: print("voltage:", vol)


def return_ctrl_data():
    T_TYPE, T_FUNC, T_LEN = 0x01, 0x10, 0x08
    s, f, st, h = g_step_control, g_pace_freq, g_car_stabilize_state, g_height
    chk = (T_TYPE + T_FUNC + T_LEN + s + f + st + h) % 256
    _tcp_send("$%02x%02x%02x%02x%02x%02x%02x%02x#" % (T_TYPE, T_FUNC, T_LEN, s, f, st, h, chk))


def return_posture_data():
    T_TYPE, T_FUNC, T_LEN = 0x01, 0x20, 0x06
    h, sh = g_height, int2hex(g_shoulder)
    chk = (T_TYPE + T_FUNC + T_LEN + h + sh) % 256
    _tcp_send("$%02x%02x%02x%02x%02x%02x#" % (T_TYPE, T_FUNC, T_LEN, h, sh, chk))


def return_action_data():
    T_TYPE, T_FUNC, T_LEN = 0x01, 0x30, 0x06
    c = g_action_continuous
    chk = (T_TYPE + T_FUNC + T_LEN + c) % 256
    _tcp_send("$%02x%02x%02x%02x%02x%02x#" % (T_TYPE, T_FUNC, T_LEN, c, 0, chk))


def return_motor_data():
    T_TYPE, T_FUNC, T_LEN = 0x01, 0x40, 0x1A
    angles = [int2hex(int(a)) for a in g_dog.read_motor()]
    chk = (T_TYPE + T_FUNC + T_LEN + sum(angles)) % 256
    vals = ''.join('%02x' % a for a in angles)
    _tcp_send("$%02x%02x%02x%s%02x#" % (T_TYPE, T_FUNC, T_LEN, vals, chk))


def _tcp_send(data):
    try:
        g_socket.send(data.encode('utf-8'))
        if g_debug: print('tcp send:', data)
    except Exception:
        pass


# ── threads ───────────────────────────────────────────────────────────────────

def task_press_up_handle():
    global g_press_up
    state_count = index = 0
    while True:
        if g_press_up:
            if state_count == 1:  g_dog.translation('z', 75)
            elif state_count == 5: g_dog.translation('z', 100)
            elif state_count == 10:
                state_count = 0; index += 1
            state_count += 1
            if index > 5:
                g_press_up = 0; state_count = index = 0
                g_dog.action(0xff)
        else:
            if state_count > 0:
                state_count = index = 0
        time.sleep(.15)


def task_joystick_handle():
    js = Dogzilla_Joystick(g_dog, debug=g_debug)
    while True:
        state = js.joystick_handle()
        if state != js.STATE_OK:
            if state == js.STATE_KEY_BREAK:
                break
            time.sleep(1); js.reconnect()


def task_oled_handle():
    if g_debug: print('Start OLED Task')
    while True:
        g_oled.main_program()
        time.sleep(1)


# ── parseur protocole TCP (identique à app_dogzilla.py) ─────────────────────

def parse_data(data):
    global g_mode, g_motor_speed, g_step_control, g_pace_freq
    global g_car_stabilize_state, g_height, g_action_continuous
    global g_shoulder, g_press_up

    data_size = len(data)
    if data_size < 8:
        return
    if hex2int(data[5:7]) != data_size - 8:
        return

    checknum = 0
    num_checknum = hex2int(data[data_size-3:data_size-1])
    for i in range(0, data_size - 4, 2):
        checknum = (hex2int(data[1+i:3+i]) + checknum) % 256
    if checknum != num_checknum:
        return

    cmd = data[3:5]

    if cmd == '0F':
        func = hex2int(data[7:9])
        g_mode = 'Home'
        if func == 0:   return_battery_voltage(); dogzilla_reset()
        elif func == 1: return_ctrl_data();   g_mode = 'Standard'
        elif func == 2: return_posture_data(); g_mode = 'Fullscreen'
        elif func == 3: return_action_data()
        elif func == 4: return_motor_data();  g_dog.motor_speed(50)
        elif func == 5: dogzilla_leg_reset(); g_dog.motor_speed(50)

    elif cmd == '02':
        return_battery_voltage()

    elif cmd == '11':
        nx = hex2int(data[7:9], False)
        ny = hex2int(data[9:11], False)
        g_dog.move_x(int( ny / 100.0 * STEP_SCALE_X * g_step_control))
        g_dog.move_y(int(-nx / 100.0 * STEP_SCALE_Y * g_step_control))
        if g_car_stabilize_state == 1:
            g_car_stabilize_state = 0; return_ctrl_data()

    elif cmd == '12':
        d = hex2int(data[7:9])
        step_x = max(5, int(STEP_SCALE_X * g_step_control))
        step_y = int(STEP_SCALE_Y * g_step_control)
        step_z = int(my_map(g_step_control, 20, 100, 30, STEP_SCALE_Z * 100))
        if   d == 1: g_dog.forward(step_x)
        elif d == 2: g_dog.back(step_x)
        elif d == 3: g_dog.left(step_y)
        elif d == 4: g_dog.right(step_y)
        elif d == 5: g_dog.turnleft(step_z)
        elif d == 6: g_dog.turnright(step_z)
        elif d == 7: dogzilla_reset(); return_ctrl_data()
        elif d == 0: g_dog.stop()
        if g_car_stabilize_state == 1 and 0 < d < 7:
            g_car_stabilize_state = 0; return_ctrl_data()

    elif cmd == '13':
        w = hex2int(data[7:9])
        g_step_control = max(20, min(100, w))

    elif cmd == '14':
        f = hex2int(data[7:9])
        if 0 < f < 4:
            g_pace_freq = f
            if f == 1:   g_dog.pace('slow');   g_dog.translation('z', 75)
            elif f == 2: g_dog.pace('normal'); g_dog.translation('z', g_height)
            elif f == 3: g_dog.pace('high');   g_dog.translation('z', g_height)

    elif cmd == '15':
        s = hex2int(data[7:9])
        g_car_stabilize_state = 1 if s > 0 else 0
        g_dog.imu(g_car_stabilize_state)

    elif cmd == '21':
        nx = hex2int(data[7:9], False)
        ny = hex2int(data[9:11], False)
        g_dog.attitude(['r', 'p'], [int(nx / 5), -int(ny / 6.6)])

    elif cmd == '22':
        h = hex2int(data[7:9])
        if g_pace_freq > 1 and 75 < h < 115:
            g_height = min(110, h)
            g_dog.translation('z', g_height)

    elif cmd == '23':
        s = hex2int(data[7:9], False)
        if -11 < s < 11:
            g_shoulder = -s
            g_dog.attitude('y', g_shoulder)

    elif cmd == '31':
        a = hex2int(data[7:9])
        if a == 0:
            dogzilla_reset()
        elif a == 20:
            g_press_up = 1
        else:
            g_press_up = 0; g_dog.action(a)

    elif cmd == '32':
        c = hex2int(data[7:9])
        g_action_continuous = c
        g_dog.perform(c)

    elif cmd == '33':
        r = hex2int(data[7:9])
        if r == 1:   dogzilla_leg_reset()
        elif r == 2: dogzilla_reset()

    elif cmd == '41':
        pass  # servo individuel — non exposé en ROS 2

    elif cmd == '51':
        pass  # patte individuelle — non exposé en ROS 2

    elif cmd == 'AA':
        pass  # calibration — volontairement désactivé


# ── serveur TCP ───────────────────────────────────────────────────────────────

def start_tcp_server(ip, port):
    global g_init, g_tcp_except_count, g_socket, g_mode
    g_init = True
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((ip, port)); sock.listen(1)

    while True:
        print('Waiting for the client to connect!')
        g_tcp_except_count = 0; times = 0
        g_socket, address = sock.accept()
        print('Connected, Client IP:', address)
        while True:
            try:
                if times == 0:
                    return_battery_voltage(); times = 1
                cmd = g_socket.recv(1024).decode('utf-8')
                if not cmd:
                    break
                i1, i2 = cmd.rfind('$'), cmd.rfind('#')
                if i1 < 0 or i2 <= i1:
                    continue
                parse_data(cmd[i1:i2+1])
                g_tcp_except_count = 0
            except Exception:
                g_tcp_except_count += 1
                if g_tcp_except_count >= 10:
                    g_tcp_except_count = 0; break
        print('socket disconnected!')
        g_socket.close(); g_mode = 'Home'


def init_tcp_socket():
    global g_tcp_ip
    if g_init:
        return
    while True:
        ip = get_ip_address()
        if ip != 'x.x.x.x':
            g_tcp_ip = ip
            print('TCP Service IP=', ip)
            break
        time.sleep(.5)
    threading.Thread(target=start_tcp_server, args=(ip, 6000),
                     name='task_tcp', daemon=True).start()


# ── flux caméra ───────────────────────────────────────────────────────────────

def mode_handle():
    global g_mode, g_camera
    while True:
        m_fps = 0; t_start = time.time()
        while True:
            if g_mode in ('Standard', 'Fullscreen'):
                ok, frame = g_camera.get_frame()
                if not ok:
                    m_fps = 0; t_start = time.time()
                    g_camera.reconnect(); time.sleep(.5); continue
                m_fps += 1
                fps = m_fps / (time.time() - t_start)
                cv.putText(frame, 'FPS:' + str(int(fps)), (10, 25),
                           cv.FONT_HERSHEY_TRIPLEX, 0.8, (0, 200, 0), 1)
                _, enc = cv.imencode('.jpg', frame)
                if enc is not None:
                    yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n'
                           + enc.tobytes() + b'\r\n')
            else:
                time.sleep(.1); m_fps = 0; t_start = time.time()


# ── Flask routes ──────────────────────────────────────────────────────────────

@app.route('/')
def index():
    return render_template('index.html')


@app.route('/video_feed')
def video_feed():
    return Response(mode_handle(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/init')
def init():
    init_tcp_socket()
    return render_template('init.html')


@app.route('/stream')
def stream():
    global g_mode
    g_mode = 'Standard'
    return '<html><body style="margin:0;background:#000">' \
           '<img src="/video_feed" style="width:100%;max-width:720px"></body></html>'


# ── main ──────────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    threading.Thread(target=task_press_up_handle, name='task_press_up', daemon=True).start()
    threading.Thread(target=task_joystick_handle, name='task_joystick', daemon=True).start()

    status = os.popen(
        "sudo systemctl status yahboom_oled.service | grep 'Active' | awk '{print $2}'"
    ).read().strip()
    if status != 'active':
        threading.Thread(target=task_oled_handle, name='task_oled', daemon=True).start()

    init_tcp_socket()

    # animation de démarrage via ROS 2
    g_dog.motor_speed(50)
    time.sleep(.1)
    g_dog.action(14)  # Stretch
    print('Waiting for connect to the APP! (ROS 2 mode)')

    try:
        server = pywsgi.WSGIServer(('0.0.0.0', 6500), app)
        server.serve_forever()
    except KeyboardInterrupt:
        g_oled.clear(True)
        rclpy.shutdown()
        print('-----program end-----')
