#!/usr/bin/env python3
"""
Dogzilla pygame teleoperation node.

Keyboard:
  Z / ↑       Forward          A       Turn left
  S / ↓       Backward         E       Turn right
  Q / ←       Strafe left      Space   Stop
  D / →       Strafe right

  1-9   Action (Lie Down … Handshake)
  0     Reset  (action 255)

  F1    Pace slow
  F2    Pace normal
  F3    Pace high

Sliders (mouse):  Translation X/Y/Z  ·  Attitude Roll/Pitch/Yaw
"""

import sys
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Int32, String

try:
    import pygame
except ImportError:
    sys.exit('pygame requis — pip install pygame')

# ── speeds published on /cmd_vel ──────────────────────────────────────────────
VX = 0.5
VY = 0.45
WZ = 1.0
CMD_HZ = 10

# ── window ────────────────────────────────────────────────────────────────────
W, H = 860, 645
FPS = 30

# ── palette ───────────────────────────────────────────────────────────────────
BG       = (28,  28,  28)
PANEL    = (42,  42,  42)
ACTIVE   = (0,  170,  80)
INACTIVE = (65,  65,  65)
WARN     = (200, 130,  0)
DANGER   = (190,  40,  40)
TEXT     = (230, 230, 230)
DIM      = (110, 110, 110)
WHITE    = (255, 255, 255)
TRACK    = (55,  55,  55)
THUMB    = (90,  90,  90)

ACTIONS = [
    (1,  '1 Lie Down'),   (2,  '2 Stand Up'),  (3,  '3 Crawl'),
    (4,  '4 Turn Around'),(5,  '5 Mark Time'),  (6,  '6 Squat'),
    (7,  '7 Roll'),       (8,  '8 Pitch'),      (9,  '9 Yaw'),
    (10, '  3-Axis'),     (11, '  Pee'),         (12, '  Sit Down'),
    (13, '  Wave'),       (14, '  Stretch'),     (15, '  Wave Body'),
    (16, '  Swing'),      (17, '  Pray'),        (18, '  Seek'),
    (19, '  Handshake'),
]


# ── ROS 2 node (runs in a background thread) ──────────────────────────────────

class TeleopPublisher(Node):
    def __init__(self):
        super().__init__('dogzilla_teleop')
        self._cmd   = self.create_publisher(Twist,   'cmd_vel',             10)
        self._act   = self.create_publisher(Int32,   'dogzilla/action',     10)
        self._pace  = self.create_publisher(String,  'dogzilla/pace',       10)
        self._trans = self.create_publisher(Vector3, 'dogzilla/translation',10)
        self._att   = self.create_publisher(Vector3, 'dogzilla/attitude',   10)

    def send_vel(self, vx, vy, wz):
        msg = Twist()
        msg.linear.x  = float(vx)
        msg.linear.y  = float(vy)
        msg.angular.z = float(wz)
        self._cmd.publish(msg)

    def send_action(self, action_id):
        msg = Int32()
        msg.data = int(action_id)
        self._act.publish(msg)

    def send_pace(self, mode: str):
        msg = String()
        msg.data = mode
        self._pace.publish(msg)

    def send_translation(self, x, y, z):
        msg = Vector3()
        msg.x, msg.y, msg.z = float(x), float(y), float(z)
        self._trans.publish(msg)

    def send_attitude(self, roll, pitch, yaw):
        msg = Vector3()
        msg.x, msg.y, msg.z = float(roll), float(pitch), float(yaw)
        self._att.publish(msg)


# ── helpers ───────────────────────────────────────────────────────────────────

def _btn(surf, rect, label, color, font):
    pygame.draw.rect(surf, color, rect, border_radius=6)
    t = font.render(label, True, WHITE)
    surf.blit(t, t.get_rect(center=rect.center))


def _label(surf, text, x, y, font, color=DIM):
    surf.blit(font.render(text, True, color), (x, y))


class Slider:
    def __init__(self, x, y, w, label, vmin, vmax, default):
        self.rect  = pygame.Rect(x, y, w, 14)
        self.label = label
        self.vmin  = vmin
        self.vmax  = vmax
        self.value = default
        self._drag = False

    def draw(self, surf, font):
        pygame.draw.rect(surf, TRACK, self.rect, border_radius=4)
        ratio = (self.value - self.vmin) / (self.vmax - self.vmin)
        tx = int(self.rect.x + ratio * self.rect.w)
        pygame.draw.circle(surf, THUMB, (tx, self.rect.centery), 9)
        lbl = font.render(f'{self.label}: {self.value:+.0f}', True, TEXT)
        surf.blit(lbl, (self.rect.x, self.rect.y - 18))

    def handle(self, event):
        if event.type == pygame.MOUSEBUTTONDOWN and self.rect.inflate(0, 20).collidepoint(event.pos):
            self._drag = True
        if event.type == pygame.MOUSEBUTTONUP:
            self._drag = False
        if event.type == pygame.MOUSEMOTION and self._drag:
            ratio = (event.pos[0] - self.rect.x) / self.rect.w
            ratio = max(0.0, min(1.0, ratio))
            self.value = round(self.vmin + ratio * (self.vmax - self.vmin))
            return True
        return False


# ── main app ──────────────────────────────────────────────────────────────────

def run_app(node: TeleopPublisher):
    pygame.init()
    screen = pygame.display.set_mode((W, H))
    pygame.display.set_caption('Dogzilla Teleop')
    clock  = pygame.time.Clock()
    font_s = pygame.font.SysFont('monospace', 13)
    font_m = pygame.font.SysFont('monospace', 15, bold=True)
    font_l = pygame.font.SysFont('monospace', 18, bold=True)

    # ── state ─────────────────────────────────────────────────────────────────
    pace = 'normal'
    last_action = None
    vx_cur = vy_cur = wz_cur = 0.0
    cmd_tick = 0

    # ── sliders ───────────────────────────────────────────────────────────────
    SX = 460
    sliders = {
        'tx': Slider(SX, 135, 360, 'Trans X', -35, 35,   0),
        'ty': Slider(SX, 185, 360, 'Trans Y', -18, 18,   0),
        'tz': Slider(SX, 235, 360, 'Trans Z',  75, 115, 105),
        'ar': Slider(SX, 335, 360, 'Roll',    -20, 20,   0),
        'ap': Slider(SX, 385, 360, 'Pitch',   -15, 15,   0),
        'ay': Slider(SX, 435, 360, 'Yaw',     -11, 11,   0),
    }
    prev_sl = {k: s.value for k, s in sliders.items()}

    # ── action buttons (5 columns) ────────────────────────────────────────────
    AX, AY, AW, AH, AGAP = 20, 468, 84, 30, 6
    act_rects = []
    for i, (aid, label) in enumerate(ACTIONS):
        col, row = i % 5, i // 5
        r = pygame.Rect(AX + col*(AW+AGAP), AY + row*(AH+AGAP), AW, AH)
        act_rects.append((r, aid, label))

    reset_rect = pygame.Rect(AX + 4*(AW+AGAP), AY + 3*(AH+AGAP), AW, AH)

    # ── pace buttons ──────────────────────────────────────────────────────────
    pace_rects = {
        'slow':   pygame.Rect(20,  52, 90, 30),
        'normal': pygame.Rect(118, 52, 90, 30),
        'high':   pygame.Rect(216, 52, 90, 30),
    }

    # ── directional d-pad ─────────────────────────────────────────────────────
    DX, DY = 60, 130
    DIR_W, DIR_H = 84, 40
    dpad = {
        'fwd':   pygame.Rect(DX + DIR_W + 6, DY,              DIR_W, DIR_H),
        'back':  pygame.Rect(DX + DIR_W + 6, DY + DIR_H*2+12, DIR_W, DIR_H),
        'left':  pygame.Rect(DX,             DY + DIR_H + 6,  DIR_W, DIR_H),
        'right': pygame.Rect(DX + (DIR_W+6)*2, DY + DIR_H + 6, DIR_W, DIR_H),
        'stop':  pygame.Rect(DX + DIR_W + 6, DY + DIR_H + 6,  DIR_W, DIR_H),
        'tl':    pygame.Rect(DX,             DY,               DIR_W, DIR_H),
        'tr':    pygame.Rect(DX + (DIR_W+6)*2, DY,             DIR_W, DIR_H),
    }

    DIR_VEL = {
        'fwd':  ( VX,   0,   0),
        'back': (-VX,   0,   0),
        'left': (  0,  VY,   0),
        'right':(  0, -VY,   0),
        'tl':   (  0,   0,  WZ),
        'tr':   (  0,   0, -WZ),
        'stop': (  0,   0,   0),
    }
    DIR_LABEL = {
        'fwd': 'Z/↑ Fwd', 'back': 'S/↓ Back',
        'left': 'Q/← Left', 'right': 'D/→ Right',
        'stop': 'Space Stop', 'tl': 'A TurnL', 'tr': 'E TurnR',
    }

    def set_vel(vx, vy, wz):
        nonlocal vx_cur, vy_cur, wz_cur
        vx_cur, vy_cur, wz_cur = vx, vy, wz

    running = True
    while running:
        dt = clock.tick(FPS)
        keys = pygame.key.get_pressed()

        # ── continuous cmd_vel from held keys ─────────────────────────────────
        cmd_tick += dt
        if cmd_tick >= 1000 / CMD_HZ:
            cmd_tick = 0
            vx = vy = wz = 0.0
            if keys[pygame.K_z] or keys[pygame.K_UP]:    vx += VX
            if keys[pygame.K_s] or keys[pygame.K_DOWN]:  vx -= VX
            if keys[pygame.K_q] or keys[pygame.K_LEFT]:  vy += VY
            if keys[pygame.K_d] or keys[pygame.K_RIGHT]: vy -= VY
            if keys[pygame.K_a]:  wz += WZ
            if keys[pygame.K_e]:  wz -= WZ
            if vx != vx_cur or vy != vy_cur or wz != wz_cur:
                set_vel(vx, vy, wz)
                node.send_vel(vx, vy, wz)

        # ── slider changes ────────────────────────────────────────────────────
        tx, ty, tz = sliders['tx'].value, sliders['ty'].value, sliders['tz'].value
        ar, ap, ay = sliders['ar'].value, sliders['ap'].value, sliders['ay'].value
        if (tx, ty, tz) != (prev_sl['tx'], prev_sl['ty'], prev_sl['tz']):
            node.send_translation(tx, ty, tz)
            prev_sl.update({'tx': tx, 'ty': ty, 'tz': tz})
        if (ar, ap, ay) != (prev_sl['ar'], prev_sl['ap'], prev_sl['ay']):
            node.send_attitude(ar, ap, ay)
            prev_sl.update({'ar': ar, 'ap': ap, 'ay': ay})

        # ── events ────────────────────────────────────────────────────────────
        sl_changed = False
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            for s in sliders.values():
                if s.handle(event):
                    sl_changed = True

            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    set_vel(0, 0, 0)
                    node.send_vel(0, 0, 0)
                elif event.key == pygame.K_F1:
                    pace = 'slow';   node.send_pace(pace)
                elif event.key == pygame.K_F2:
                    pace = 'normal'; node.send_pace(pace)
                elif event.key == pygame.K_F3:
                    pace = 'high';   node.send_pace(pace)
                elif event.key == pygame.K_0:
                    last_action = 'Reset'; node.send_action(255)
                else:
                    for k, aid in [(pygame.K_1,1),(pygame.K_2,2),(pygame.K_3,3),
                                   (pygame.K_4,4),(pygame.K_5,5),(pygame.K_6,6),
                                   (pygame.K_7,7),(pygame.K_8,8),(pygame.K_9,9)]:
                        if event.key == k:
                            last_action = next(l for (a,l) in ACTIONS if a == aid).strip()
                            node.send_action(aid)

            if event.type == pygame.MOUSEBUTTONDOWN:
                mp = event.pos
                for mode, r in pace_rects.items():
                    if r.collidepoint(mp):
                        pace = mode; node.send_pace(pace)
                for r, aid, label in act_rects:
                    if r.collidepoint(mp):
                        last_action = label.strip(); node.send_action(aid)
                if reset_rect.collidepoint(mp):
                    last_action = 'Reset'; node.send_action(255)
                for dk, dr in dpad.items():
                    if dr.collidepoint(mp):
                        v = DIR_VEL[dk]; set_vel(*v); node.send_vel(*v)

        # ── draw ──────────────────────────────────────────────────────────────
        screen.fill(BG)

        # title
        _label(screen, 'DOGZILLA  TELEOP', 20, 14, font_l, WHITE)

        # pace
        _label(screen, 'Pace:', 20, 58, font_s, DIM)
        for mode, r in pace_rects.items():
            c = ACTIVE if pace == mode else INACTIVE
            _btn(screen, r, mode.upper(), c, font_m)
        pygame.draw.rect(screen, DANGER, pygame.Rect(320, 52, 90, 30), border_radius=6)
        _label(screen, '0 Reset', 334, 62, font_s, WHITE)

        # d-pad
        _label(screen, 'Movement', 20, 108, font_s, DIM)
        for dk, dr in dpad.items():
            active = (dk == 'stop' and vx_cur == 0 and vy_cur == 0 and wz_cur == 0) or \
                     (vx_cur, vy_cur, wz_cur) == DIR_VEL[dk]
            c = ACTIVE if (active and dk != 'stop') else (WARN if dk == 'stop' else INACTIVE)
            _btn(screen, dr, DIR_LABEL[dk], c, font_s)

        # status bar
        sx, sy = 20, 310
        pygame.draw.rect(screen, PANEL, pygame.Rect(sx, sy, 400, 70), border_radius=8)
        _label(screen, f'vx {vx_cur:+.2f}  vy {vy_cur:+.2f}  wz {wz_cur:+.2f}', sx+12, sy+10, font_m, TEXT)
        _label(screen, f'pace: {pace}', sx+12, sy+38, font_s, TEXT)
        if last_action:
            _label(screen, f'last: {last_action}', sx+150, sy+38, font_s, TEXT)

        # sliders — translation
        pygame.draw.rect(screen, PANEL, pygame.Rect(SX-10, 90, 400, 170), border_radius=8)
        _label(screen, 'Translation (mm)', SX, 96, font_s, DIM)
        for k in ('tx','ty','tz'):
            sliders[k].draw(screen, font_s)

        # sliders — attitude
        pygame.draw.rect(screen, PANEL, pygame.Rect(SX-10, 290, 400, 170), border_radius=8)
        _label(screen, 'Attitude (°)', SX, 296, font_s, DIM)
        for k in ('ar','ap','ay'):
            sliders[k].draw(screen, font_s)

        # actions grid
        _label(screen, 'Actions', 20, 422, font_s, DIM)
        for r, aid, label in act_rects:
            _btn(screen, r, label.strip(), INACTIVE, font_s)
        _btn(screen, reset_rect, 'RESET', DANGER, font_s)

        # keybindings reminder
        _label(screen, 'F1/F2/F3: pace   0: reset   1-9: actions   Space: stop', 460, 622, font_s, DIM)

        pygame.display.flip()

    pygame.quit()


def main(args=None):
    rclpy.init(args=args)
    node = TeleopPublisher()
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()
    try:
        run_app(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
