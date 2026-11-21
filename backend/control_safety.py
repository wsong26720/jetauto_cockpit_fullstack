"""Single-owner manual velocity lease. No ROS or hardware dependencies."""
import math
import threading
import time


class ControlError(Exception):
    def __init__(self, status, detail):
        super().__init__(detail)
        self.status = status
        self.detail = detail


class VelocityLease:
    def __init__(self, publish, ready, clock=time.monotonic, timeout=0.5):
        self.publish = publish
        self.ready = ready
        self.clock = clock
        self.timeout = timeout
        self.lock = threading.RLock()
        self.owner = None
        self.deadline = 0.0
        self.command = (0.0, 0.0, 0.0)
        self.estop = False
        self.stop_ticks = 0

    def _stop(self):
        self.command = (0.0, 0.0, 0.0)
        self.owner = None
        self.stop_ticks = 4
        self.publish(0.0, 0.0, 0.0)

    def command_from(self, owner, x, y, yaw):
        with self.lock:
            values = (x, y, yaw)
            if not all(math.isfinite(v) for v in values):
                raise ControlError(422, "速度必须为有限数值")
            if any(abs(v) > lim for v, lim in zip(values, (0.2, 0.2, 0.5))):
                raise ControlError(422, "速度超出限幅")
            if self.estop:
                raise ControlError(423, "软急停已锁定")
            now = self.clock()
            if self.owner is not None and now >= self.deadline:
                self._stop()
            if self.owner is not None and self.owner != owner:
                raise ControlError(409, "另一控制客户端正在控制底盘")
            if not self.ready():
                if self.owner is not None:
                    self._stop()
                raise ControlError(503, "ROS 未就绪或 /cmd_vel 没有底盘订阅者")
            if not any(values):
                self._stop()
                return
            self.owner = owner
            self.command = values
            self.deadline = now + self.timeout
            self.stop_ticks = 0
            self.publish(*values)

    def tick(self):
        with self.lock:
            if self.estop:
                self.publish(0.0, 0.0, 0.0)
            elif self.owner is not None:
                if self.clock() >= self.deadline or not self.ready():
                    self._stop()
                else:
                    self.publish(*self.command)
            elif self.stop_ticks:
                self.publish(0.0, 0.0, 0.0)
                self.stop_ticks -= 1

    def disconnect(self, owner):
        with self.lock:
            if self.owner == owner:
                self._stop()

    def stop_manual(self):
        """Revoke manual motion without creating a new estop latch."""
        with self.lock:
            self._stop()

    def set_estop(self, engage):
        with self.lock:
            self.estop = engage
            if engage:
                self._stop()
            # Releasing never restores the previous command.

    def snapshot(self):
        with self.lock:
            return {"estop_engaged": self.estop, "manual_control_active": self.owner is not None,
                    "watchdog_timeout_sec": self.timeout}


class MotionArbiter:
    """Only the gateway publishes chassis Twist. Nav2 uses a private input topic."""
    def __init__(self, manual, publish, hardware_ready, clock=time.monotonic):
        self.manual=manual;self.publish=publish;self.hardware_ready=hardware_ready;self.clock=clock
        self.owner=None;self.command=(0.0,0.0,0.0);self.deadline=0
        self.nav_ready=lambda:False
        self.localizing=False;self.localization_ready=lambda:False

    def begin(self,token,localization=False):
        with self.manual.lock:
            if self.manual.estop:raise ControlError(423,'急停已锁定')
            if self.manual.owner:raise ControlError(409,'请先松开手动控制器')
            if self.owner:raise ControlError(409,'已有导航任务占用控制权')
            self.owner=token;self.command=(0.0,0.0,0.0);self.deadline=0
            self.localizing=localization
            self.manual.stop_ticks=0

    def end(self,token=None):
        with self.manual.lock:
            if token is not None and self.owner!=token:return
            had_owner=self.owner is not None
            self.owner=None;self.command=(0.0,0.0,0.0);self.deadline=0
            self.localizing=False
            if had_owner:self.manual._stop()

    def receive(self,x,y,yaw):
        with self.manual.lock:
            if self.owner is None or self.manual.estop or self.localizing:return
            if not all(math.isfinite(v) for v in (x,y,yaw)):return
            self.command=(max(-0.1,min(0.15,x)),max(-0.1,min(0.1,y)),max(-0.4,min(0.4,yaw)))
            self.deadline=self.clock()+0.5

    def localization_command(self,token,yaw):
        with self.manual.lock:
            if not self.localizing or self.owner!=token or self.manual.estop:return
            if not math.isfinite(yaw):return
            self.command=(0.,0.,max(-.10,min(.10,yaw)));self.deadline=self.clock()+.3

    def tick(self):
        with self.manual.lock:
            if self.manual.estop or self.manual.owner or self.manual.stop_ticks:
                self.manual.tick()
            elif self.owner is not None:
                permitted=self.localization_ready() if self.localizing else self.nav_ready()
                allowed=self.clock()<self.deadline and self.hardware_ready() and permitted
                self.publish(*(self.command if allowed else (0.0,0.0,0.0)))

    def snapshot(self):
        with self.manual.lock:
            return {'navigation_control_active':self.owner is not None,
                    'localization_control_active':self.owner is not None and self.localizing,
                    'source':'estop' if self.manual.estop else ('manual' if self.manual.owner else ('localization' if self.localizing and self.owner else ('navigation' if self.owner else 'idle')))}
