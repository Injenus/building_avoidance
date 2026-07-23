#!/usr/bin/env python3
"""Автономный облёт препятствия для ArduCopter.

Алгоритм — Bug2-подобный: прямой полёт к цели, при блокировке коридора
переход в обход границы препятствия с постоянным зазором, возврат к цели
после освобождения коридора. Все расчёты — в локальной ENU (MAVROS).
"""
import math
from collections import deque
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Twist
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode

RATE = 20.0


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class Phase(Enum):
    WAIT = 'ожидание связи'
    MODE = 'перевод в GUIDED'
    ARM = 'арминг'
    TAKEOFF = 'взлёт'
    GO = 'полёт к цели'
    AVOID = 'обход препятствия'
    HOLD = 'цель достигнута'
    LAND = 'посадка'
    FAILSAFE = 'нет данных лидара'


class Planner(Node):
    def __init__(self):
        super().__init__('avoidance_planner')
        d = {
            'target_x': 0.0, 'target_y': 60.0, 'cruise_alt': 5.0,
            'cruise_speed': 3.0, 'goal_tolerance': 2.0, 'clearance': 6.0,
            'detect_dist': 15.0, 'corridor_half': 3.0, 'max_range': 20.0,
            'stuck_time': 5.0, 'stuck_progress': 1.0, 'lidar_timeout': 1.5,
            'finish_action': 'land',
        }
        for k, v in d.items():
            self.declare_parameter(k, v)
        g = lambda k: self.get_parameter(k).value
        self.tgt = (g('target_x'), g('target_y'))
        self.alt, self.spd = g('cruise_alt'), g('cruise_speed')
        self.tol, self.clr = g('goal_tolerance'), g('clearance')
        self.det, self.half = g('detect_dist'), g('corridor_half')
        self.rmax = g('max_range')
        self.st_t, self.st_p = g('stuck_time'), g('stuck_progress')
        self.lid_to, self.finish = g('lidar_timeout'), g('finish_action')

        self.state = None
        self.pos = None
        self.yaw = 0.0
        self.scan = None
        self.scan_t = 0.0
        self.phase = Phase.WAIT
        self.side = 1          # +1 обход влево, -1 вправо
        self.hit_dist = None   # дальность до цели в момент входа в обход
        self.hist = deque()
        self.t_phase = self.now()
        self.escapes = 0

        q = qos_profile_sensor_data
        self.create_subscription(State, '/mavros/state', self.cb_state, q)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.cb_pose, q)
        self.create_subscription(LaserScan, '/scan', self.cb_scan, q)
        self.pub = self.create_publisher(
            Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)

        self.cli_mode = self.create_client(SetMode, '/mavros/set_mode')
        self.cli_arm = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.cli_to = self.create_client(CommandTOL, '/mavros/cmd/takeoff')

        self.create_timer(1.0 / RATE, self.loop)
        self.get_logger().info('цель ENU: x=%.1f y=%.1f' % self.tgt)

    # ---------- вспомогательное ----------
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def cb_state(self, m):
        self.state = m

    def cb_pose(self, m):
        p, o = m.pose.position, m.pose.orientation
        self.pos = (p.x, p.y, p.z)
        self.yaw = math.atan2(2.0 * (o.w * o.z + o.x * o.y),
                              1.0 - 2.0 * (o.y ** 2 + o.z ** 2))

    def cb_scan(self, m):
        self.scan, self.scan_t = m, self.now()

    def set_phase(self, ph):
        if ph != self.phase:
            self.get_logger().info('%s -> %s' % (self.phase.value, ph.value))
            self.phase, self.t_phase = ph, self.now()

    def send(self, vx, vy, vz=0.0, wz=0.0):
        t = Twist()
        t.linear.x, t.linear.y, t.linear.z = vx, vy, vz
        t.angular.z = wz
        self.pub.publish(t)

    # ---------- геометрия ----------
    def points(self):
        """Точки лидара в мировой ENU: (x, y, дальность, азимут)."""
        out = []
        if self.scan is None:
            return out
        s, a = self.scan, self.scan.angle_min
        for r in s.ranges:
            if math.isfinite(r) and s.range_min < r < min(s.range_max, self.rmax):
                th = wrap(self.yaw + a)
                out.append((self.pos[0] + r * math.cos(th),
                            self.pos[1] + r * math.sin(th), r, th))
            a += s.angle_increment
        return out

    def blocked(self, pts, direction):
        """Есть ли препятствие в прямоугольном коридоре по курсу."""
        ux, uy = math.cos(direction), math.sin(direction)
        lat = []
        for px, py, _, _ in pts:
            vx, vy = px - self.pos[0], py - self.pos[1]
            along = vx * ux + vy * uy
            side = -vx * uy + vy * ux
            if 0.0 < along < self.det and abs(side) < self.half:
                lat.append(side)
        return (len(lat) > 0), lat

    # ---------- основной цикл ----------
    def loop(self):
        if self.state is None or self.pos is None:
            return

        flying = self.phase in (Phase.GO, Phase.AVOID)
        if flying and self.now() - self.scan_t > self.lid_to:
            self.set_phase(Phase.FAILSAFE)

        dx, dy = self.tgt[0] - self.pos[0], self.tgt[1] - self.pos[1]
        dist = math.hypot(dx, dy)
        gdir = math.atan2(dy, dx)
        vz = clamp((self.alt - self.pos[2]) * 1.0, -1.0, 1.0)

        if self.phase is Phase.WAIT:
            if self.state.connected:
                self.set_phase(Phase.MODE)

        elif self.phase is Phase.MODE:
            if self.state.mode == 'GUIDED':
                self.set_phase(Phase.ARM)
            elif self.now() - self.t_phase > 1.0:
                self.cli_mode.call_async(SetMode.Request(custom_mode='GUIDED'))
                self.t_phase = self.now()

        elif self.phase is Phase.ARM:
            if self.state.armed:
                self.cli_to.call_async(CommandTOL.Request(altitude=float(self.alt)))
                self.set_phase(Phase.TAKEOFF)
            elif self.now() - self.t_phase > 1.0:
                self.cli_arm.call_async(CommandBool.Request(value=True))
                self.t_phase = self.now()

        elif self.phase is Phase.TAKEOFF:
            if self.pos[2] > self.alt * 0.9:
                self.hist.clear()
                self.set_phase(Phase.GO)

        elif self.phase is Phase.GO:
            if dist < self.tol:
                self.set_phase(Phase.HOLD)
                return
            pts = self.points()
            blk, lat = self.blocked(pts, gdir)
            if blk:
                self.side = 1 if (sum(lat) / len(lat)) < 0 else -1
                self.hit_dist = dist
                self.escapes = 0
                self.hist.clear()
                self.set_phase(Phase.AVOID)
            else:
                self.send(self.spd * math.cos(gdir), self.spd * math.sin(gdir), vz)
            self.check_stuck(dist)

        elif self.phase is Phase.AVOID:
            if dist < self.tol:
                self.set_phase(Phase.HOLD)
                return
            pts = self.points()
            if not pts:
                # Нет отражений: препятствие потеряно из виду — возврат к цели,
                # но зависать нельзя, поэтому продолжаем считать прогресс.
                self.send(self.spd * math.cos(gdir), self.spd * math.sin(gdir), vz)
                self.check_stuck(dist)
                return
            blk, _ = self.blocked(pts, gdir)
            if not blk and dist < self.hit_dist - 1.0:
                self.hist.clear()
                self.set_phase(Phase.GO)
                return
            near = min(pts, key=lambda p: p[2])
            rmin, thmin = near[2], near[3]
            tang = wrap(thmin + self.side * math.pi / 2.0)
            corr = clamp((rmin - self.clr) * 0.5, -1.0, 1.0)
            ex = math.cos(tang) + corr * math.cos(thmin)
            ey = math.sin(tang) + corr * math.sin(thmin)
            n = math.hypot(ex, ey) or 1.0
            self.send(self.spd * ex / n, self.spd * ey / n, vz)
            self.check_stuck(dist)

        elif self.phase is Phase.FAILSAFE:
            self.send(0.0, 0.0, vz)
            if self.now() - self.scan_t < self.lid_to:
                self.set_phase(Phase.GO)
            elif self.now() - self.t_phase > 5.0:
                self.cli_mode.call_async(SetMode.Request(custom_mode='RTL'))
                self.set_phase(Phase.LAND)

        elif self.phase is Phase.HOLD:
            self.send(0.0, 0.0, vz)
            if self.finish == 'land' and self.now() - self.t_phase > 3.0:
                self.cli_mode.call_async(SetMode.Request(custom_mode='LAND'))
                self.set_phase(Phase.LAND)

    def check_stuck(self, dist):
        """Нет продвижения к цели за окно -> смена стороны обхода и зазора."""
        t = self.now()
        self.hist.append((t, dist))
        while self.hist and t - self.hist[0][0] > self.st_t:
            old_t, old_d = self.hist.popleft()
            if old_d - dist < self.st_p:
                self.escapes += 1
                self.side *= -1
                self.clr = min(self.clr + 2.0, 15.0)
                self.hist.clear()
                self.get_logger().warn(
                    'застревание #%d: сторона=%+d зазор=%.1f м'
                    % (self.escapes, self.side, self.clr))
                if self.phase is Phase.GO:
                    self.hit_dist = dist
                    self.set_phase(Phase.AVOID)
            break


def main():
    rclpy.init()
    n = Planner()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
