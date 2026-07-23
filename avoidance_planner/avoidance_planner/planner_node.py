#!/usr/bin/env python3
"""Автономный облёт препятствия для ArduCopter (алгоритм Bug2).

m-линия — прямая старт→цель. Дрон летит по ней; при блокировке коридора
запоминает точку встречи, обходит границу препятствия с постоянным зазором,
не меняя сторону, и возвращается на m-линию, когда оказывается ближе к цели,
чем в точке встречи. Фиксированная сторона + уход только с m-линии — то, на
чём держится гарантия завершения.

Все расчёты — в локальной ENU (MAVROS): x — восток, y — север, z — вверх.
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
    GO = 'полёт по m-линии'
    AVOID = 'обход границы'
    HOLD = 'цель достигнута'
    LAND = 'посадка'
    FAILSAFE = 'нет данных лидара'


class Planner(Node):
    def __init__(self):
        super().__init__('avoidance_planner')
        d = {
            'target_x': 0.0, 'target_y': 60.0,
            'cruise_alt': 8.0, 'cruise_speed': 3.0, 'goal_tolerance': 2.0,
            'clearance': 6.0, 'detect_dist': 12.0, 'corridor_half': 2.5,
            'max_range': 15.0, 'mline_tol': 2.0, 'leave_margin': 3.0,
            'stuck_time': 6.0, 'stuck_move': 2.0, 'lidar_timeout': 1.5,
            'smooth': 0.25, 'finish_action': 'land',
        }
        for k, v in d.items():
            self.declare_parameter(k, v)
        g = lambda k: self.get_parameter(k).value
        self.tgt = (g('target_x'), g('target_y'))
        self.alt, self.spd = g('cruise_alt'), g('cruise_speed')
        self.tol = g('goal_tolerance')
        self.clr0 = self.clr = g('clearance')
        self.det, self.half = g('detect_dist'), g('corridor_half')
        self.rmax = g('max_range')
        self.mtol, self.lmargin = g('mline_tol'), g('leave_margin')
        self.st_t, self.st_move = g('stuck_time'), g('stuck_move')
        self.lid_to, self.smooth = g('lidar_timeout'), g('smooth')
        self.finish = g('finish_action')

        self.state = self.pos = self.scan = None
        self.yaw = 0.0
        self.scan_t = 0.0
        self.phase = Phase.WAIT
        self.t_phase = self.now()
        self.start = None        # начало m-линии
        self.side = 1            # +1 против часовой, -1 по часовой
        self.hit_dist = None     # расстояние до цели в точке встречи
        self.vcmd = (0.0, 0.0)
        self.hist = deque()
        self.escapes = 0
        self.t_log = 0.0

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

    # ---------- служебное ----------
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

    def stop(self, vz=0.0):
        self.vcmd = (0.0, 0.0)
        t = Twist()
        t.linear.z = vz
        self.pub.publish(t)

    def drive(self, ex, ey, vz):
        """Сглаженная подача скорости в ENU + разворот носа по движению."""
        n = math.hypot(ex, ey)
        if n < 1e-6:
            self.stop(vz)
            return
        tx, ty = self.spd * ex / n, self.spd * ey / n
        a = self.smooth
        self.vcmd = (self.vcmd[0] * (1 - a) + tx * a,
                     self.vcmd[1] * (1 - a) + ty * a)
        t = Twist()
        t.linear.x, t.linear.y, t.linear.z = self.vcmd[0], self.vcmd[1], vz
        t.angular.z = clamp(wrap(math.atan2(self.vcmd[1], self.vcmd[0]) - self.yaw) * 0.6,
                            -0.6, 0.6)
        self.pub.publish(t)

    # ---------- геометрия ----------
    def points(self):
        """Лучи лидара в мировой ENU: (x, y, дальность, азимут)."""
        out = []
        if self.scan is None:
            return out
        s, a = self.scan, self.scan.angle_min
        top = min(s.range_max, self.rmax)
        for r in s.ranges:
            if math.isfinite(r) and s.range_min < r < top:
                th = wrap(self.yaw + a)
                out.append((self.pos[0] + r * math.cos(th),
                            self.pos[1] + r * math.sin(th), r, th))
            a += s.angle_increment
        return out

    def blocked(self, pts, direction):
        """Препятствие в прямоугольном коридоре по курсу."""
        ux, uy = math.cos(direction), math.sin(direction)
        lat = []
        for px, py, _, _ in pts:
            vx, vy = px - self.pos[0], py - self.pos[1]
            along = vx * ux + vy * uy
            off = -vx * uy + vy * ux
            if 0.0 < along < self.det and abs(off) < self.half:
                lat.append(off)
        return (len(lat) > 0), lat

    def mline_offset(self):
        """Знаковое отклонение от m-линии старт->цель, м."""
        sx, sy = self.start
        dx, dy = self.tgt[0] - sx, self.tgt[1] - sy
        n = math.hypot(dx, dy) or 1.0
        return (-(self.pos[0] - sx) * dy + (self.pos[1] - sy) * dx) / n

    # ---------- цикл ----------
    def loop(self):
        if self.state is None or self.pos is None:
            return

        if self.phase in (Phase.GO, Phase.AVOID) and \
                self.now() - self.scan_t > self.lid_to:
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
                self.start = (self.pos[0], self.pos[1])   # начало m-линии
                self.hist.clear()
                self.set_phase(Phase.GO)

        elif self.phase is Phase.GO:
            if dist < self.tol:
                self.set_phase(Phase.HOLD)
                return
            pts = self.points()
            blk, lat = self.blocked(pts, gdir)
            if blk:
                # сторона выбирается ОДИН раз: обходим с той стороны,
                # где препятствия меньше
                self.side = 1 if (sum(lat) / len(lat)) < 0 else -1
                self.hit_dist = dist
                self.clr = self.clr0
                self.escapes = 0
                self.hist.clear()
                self.get_logger().info(
                    'встреча: до цели %.1f м, сторона %+d' % (dist, self.side))
                self.set_phase(Phase.AVOID)
            else:
                self.drive(math.cos(gdir), math.sin(gdir), vz)
            self.check_stuck()

        elif self.phase is Phase.AVOID:
            if dist < self.tol:
                self.set_phase(Phase.HOLD)
                return
            pts = self.points()

            # условие ухода по Bug2: снова на m-линии и ближе, чем в точке встречи
            on_m = abs(self.mline_offset()) < self.mtol
            closer = dist < self.hit_dist - self.lmargin
            clear = not self.blocked(pts, gdir)[0]
            if on_m and closer and clear and self.now() - self.t_phase > 2.0:
                self.clr = self.clr0
                self.hist.clear()
                self.set_phase(Phase.GO)
                return

            if not pts:
                # граница потеряна из виду — идём к цели, но продолжаем
                # следить за прогрессом, зависать нельзя
                self.drive(math.cos(gdir), math.sin(gdir), vz)
            else:
                near = min(pts, key=lambda p: p[2])
                rmin, thmin = near[2], near[3]
                tang = wrap(thmin + self.side * math.pi / 2.0)
                corr = clamp((rmin - self.clr) * 0.35, -0.7, 0.7)
                self.drive(math.cos(tang) + corr * math.cos(thmin),
                           math.sin(tang) + corr * math.sin(thmin), vz)
                if self.now() - self.t_log > 2.0:
                    self.t_log = self.now()
                    self.get_logger().info(
                        'обход: до стены %.1f м | до цели %.1f (H=%.1f) | от m-линии %.1f'
                        % (rmin, dist, self.hit_dist, self.mline_offset()))
            self.check_stuck()

        elif self.phase is Phase.FAILSAFE:
            self.stop(vz)
            if self.now() - self.scan_t < self.lid_to:
                self.set_phase(Phase.GO)
            elif self.now() - self.t_phase > 5.0:
                self.cli_mode.call_async(SetMode.Request(custom_mode='RTL'))
                self.set_phase(Phase.LAND)

        elif self.phase is Phase.HOLD:
            self.stop(vz)
            if self.finish == 'land' and self.now() - self.t_phase > 3.0:
                self.cli_mode.call_async(SetMode.Request(custom_mode='LAND'))
                self.set_phase(Phase.LAND)

    def check_stuck(self):
        """Застревание = дрон физически не перемещается.

        Мерить сокращение расстояния до цели нельзя: при обходе границы оно
        закономерно не уменьшается, это нормальная работа алгоритма.
        """
        t = self.now()
        self.hist.append((t, self.pos[0], self.pos[1]))
        while self.hist and t - self.hist[0][0] > self.st_t:
            _, x0, y0 = self.hist[0]
            moved = math.hypot(self.pos[0] - x0, self.pos[1] - y0)
            if moved < self.st_move:
                self.escapes += 1
                self.clr = min(self.clr + 1.5, self.clr0 + 4.0)
                self.hist.clear()
                self.get_logger().warn(
                    'застревание #%d: смещение %.1f м за %.0f с, зазор -> %.1f м'
                    % (self.escapes, moved, self.st_t, self.clr))
                # разворот стороны — крайняя мера, ломает гарантию Bug2,
                # поэтому только когда увеличение зазора не помогло дважды
                if self.escapes % 3 == 0:
                    self.side *= -1
                    self.hit_dist = math.hypot(self.tgt[0] - self.pos[0],
                                               self.tgt[1] - self.pos[1])
                    self.get_logger().warn('крайняя мера: сторона -> %+d' % self.side)
                return
            self.hist.popleft()
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
