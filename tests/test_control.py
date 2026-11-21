import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'backend'))
from control_safety import VelocityLease, ControlError


class ControlTests(unittest.TestCase):
    def setUp(self):
        self.now = 100.0
        self.ready = True
        self.output = []
        self.lease = VelocityLease(lambda *cmd: self.output.append(cmd), lambda: self.ready,
                                   clock=lambda: self.now)

    def move(self, owner='a'):
        self.lease.command_from(owner, 0.1, 0.0, 0.0)

    def test_disconnect_and_idle_do_not_continue_velocity(self):
        self.move()
        self.lease.disconnect('a')
        for _ in range(10): self.lease.tick()
        self.assertEqual(self.output[1:], [(0, 0, 0)] * 5)
        self.assertIsNone(self.lease.owner)

    def test_timeout_without_any_browser_cleanup_publishes_stop(self):
        self.move()
        self.now += 0.501
        for _ in range(10): self.lease.tick()
        self.assertEqual(self.output[1:], [(0, 0, 0)] * 5)
        size = len(self.output)
        self.now += 1000
        self.lease.tick()
        self.assertEqual(len(self.output), size)  # idle never floods other publishers

    def test_active_heartbeat_renews_lease(self):
        self.move()
        self.now += 0.4
        self.move()
        self.now += 0.4
        self.lease.tick()
        self.assertEqual(self.output[-1], (0.1, 0, 0))
        self.now += 0.101
        self.lease.tick()
        self.assertEqual(self.output[-1], (0, 0, 0))

    def test_other_browser_cannot_stop_or_steal_active_lease(self):
        self.move()
        for cmd in ((0.1, 0, 0), (0, 0, 0)):
            with self.assertRaises(ControlError) as ctx:
                self.lease.command_from('b', *cmd)
            self.assertEqual(ctx.exception.status, 409)
        self.lease.disconnect('b')
        self.assertEqual(self.lease.owner, 'a')
        self.lease.disconnect('a')
        self.move('b')
        self.assertEqual(self.lease.owner, 'b')

    def test_estop_blocks_all_clients_and_release_does_not_resume(self):
        self.move()
        self.lease.set_estop(True)
        with self.assertRaises(ControlError): self.move('b')
        self.lease.tick()
        self.lease.set_estop(False)
        for _ in range(8): self.lease.tick()
        self.assertTrue(all(cmd == (0, 0, 0) for cmd in self.output[1:]))
        self.assertIsNone(self.lease.owner)

    def test_missing_driver_and_driver_loss(self):
        self.ready = False
        with self.assertRaises(ControlError) as ctx: self.move()
        self.assertEqual(ctx.exception.status, 503)
        self.assertEqual(self.output, [])
        self.ready = True
        self.move()
        self.ready = False
        self.lease.tick()
        self.assertEqual(self.output[-1], (0, 0, 0))

    def test_invalid_speeds_never_publish(self):
        for cmd in ((float('nan'), 0, 0), (float('inf'), 0, 0), (0.21, 0, 0), (0, 0, 0.51)):
            with self.assertRaises(ControlError): self.lease.command_from('a', *cmd)
        self.assertFalse(self.output)


if __name__ == '__main__': unittest.main()
