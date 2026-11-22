"""Persist cockpit-owned costmap settings; never mutate running ROS nodes."""
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import tempfile
import threading
import time

import yaml


SCALARS = {
    'inflation_radius': 'inflation_layer.inflation_radius',
    'cost_scaling_factor': 'inflation_layer.cost_scaling_factor',
    'resolution': 'resolution',
    'obstacle_max_range': 'obstacle_layer.scan.obstacle_max_range',
    'raytrace_max_range': 'obstacle_layer.scan.raytrace_max_range',
}
PARAMETERS = ['footprint', 'footprint_padding', *SCALARS.values()]


class SettingsError(ValueError):
    def __init__(self, message, status=422):
        super().__init__(message)
        self.status = status


def _number(value, low, high, label):
    if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value):
        raise SettingsError(label + '必须是有限数字')
    if not low <= value <= high:
        raise SettingsError(f'{label}应在 {low}–{high} 之间')
    return float(value)


def validate(settings):
    if not isinstance(settings, dict) or set(settings) != {'footprint', 'footprint_padding', 'global', 'local'}:
        raise SettingsError('配置必须包含 footprint、footprint_padding、global 和 local，不能含其他字段')
    vertices = settings['footprint']
    if not isinstance(vertices, list) or not 3 <= len(vertices) <= 32:
        raise SettingsError('车体轮廓需要 3–32 个按边界顺序排列的顶点')
    points = []
    for p in vertices:
        if not isinstance(p, list) or len(p) != 2:
            raise SettingsError('每个轮廓顶点必须是 [x, y]，单位为米')
        points.append([_number(v, -2, 2, '轮廓坐标') for v in p])
    # Every other vertex and the robot origin must be strictly on one side of
    # every edge: rejects concavity, self-intersection and a misplaced origin.
    direction = None
    inscribed = float('inf')
    for i, a in enumerate(points):
        b = points[(i + 1) % len(points)]
        dx, dy = b[0] - a[0], b[1] - a[1]
        length = math.hypot(dx, dy)
        if length < 1e-6:
            raise SettingsError('车体轮廓不能含重复顶点')
        for j, p in enumerate(points + [[0., 0.]]):
            if j in (i, (i + 1) % len(points)):
                continue
            cross = dx * (p[1] - a[1]) - dy * (p[0] - a[0])
            if abs(cross) < 1e-8:
                raise SettingsError('轮廓必须是包含车体原点的严格凸多边形')
            sign = cross > 0
            if direction is not None and direction != sign:
                raise SettingsError('轮廓必须按边界顺序排列、无自交，且包含车体原点')
            direction = sign
        inscribed = min(inscribed, abs(dx * -a[1] - dy * -a[0]) / length)
    if inscribed < .05:
        raise SettingsError('车体原点到轮廓边缘至少需要 0.05 m，请检查尺寸和坐标')
    padding = _number(settings['footprint_padding'], 0, .1, '轮廓余量')
    # Nav2 Humble pads vertex coordinates along their sign, not edge normals.
    padded = [[v + math.copysign(padding, v) if v else v for v in p] for p in points]
    padded_radius = min(abs((b[0]-a[0]) * -a[1] - (b[1]-a[1]) * -a[0]) /
                        math.hypot(b[0]-a[0], b[1]-a[1])
                        for a, b in zip(padded, padded[1:] + padded[:1]))
    result = {'footprint': points, 'footprint_padding': padding}
    limits = {'inflation_radius': (.1, 2.), 'cost_scaling_factor': (.1, 30.),
              'resolution': (.02, .1), 'obstacle_max_range': (.2, 20.), 'raytrace_max_range': (.2, 25.)}
    for name in ('global', 'local'):
        row = settings[name]
        if not isinstance(row, dict) or set(row) != set(SCALARS):
            raise SettingsError(name + ' 参数字段不完整或含未知字段')
        result[name] = {key: _number(row[key], *limits[key], name + '.' + key) for key in SCALARS}
        if row['inflation_radius'] + 1e-9 < padded_radius:
            raise SettingsError(f'{name} 膨胀半径不能小于带余量轮廓的内切半径 {padded_radius:.3f} m')
        if row['raytrace_max_range'] < row['obstacle_max_range']:
            raise SettingsError(name + ' 清障距离不能小于障碍标记距离')
    return result


def _extract(params):
    row = {}
    for key, path in SCALARS.items():
        value = params
        for segment in path.split('.'):
            value = value[segment]
        row[key] = float(value)
    return row


class CostmapSettingsStore:
    def __init__(self, path):
        self.path = Path(path)
        self.lock = threading.RLock()

    def _read(self):
        try:
            raw = self.path.read_bytes()
            doc = yaml.safe_load(raw)
            params = {name: doc[name + '_costmap'][name + '_costmap']['ros__parameters'] for name in ('global', 'local')}
            profile = params['global']['footprint']
            settings = {'footprint': json.loads(profile) if isinstance(profile, str) else profile,
                        'footprint_padding': float(params['global']['footprint_padding']),
                        **{name: _extract(params[name]) for name in params}}
            warnings = []
            if params['local']['footprint'] != profile or params['local']['footprint_padding'] != params['global']['footprint_padding']:
                warnings.append('文件中的全局、局部车体轮廓不一致；保存时会统一为面板中的轮廓')
            return raw, doc, settings, warnings
        except (OSError, ValueError, TypeError, KeyError, yaml.YAMLError) as exc:
            raise SettingsError('无法读取代价地图配置文件：' + str(exc), 503) from exc

    def read(self):
        with self.lock:
            raw, _, settings, warnings = self._read()
            return {'settings': settings, 'revision': hashlib.sha256(raw).hexdigest(), 'warnings': warnings,
                    'apply_mode': 'navigation_restart', 'message': '保存只更新下次启动配置；请在机器人停稳后手动重启导航模块'}

    def save(self, settings, revision):
        settings = validate(settings)
        with self.lock:
            raw, doc, _, _ = self._read()
            if revision != hashlib.sha256(raw).hexdigest():
                raise SettingsError('配置已被其他页面或程序修改，请重新读取后再保存', 409)
            for name in ('global', 'local'):
                params = doc[name + '_costmap'][name + '_costmap']['ros__parameters']
                params['footprint'] = json.dumps(settings['footprint'], separators=(',', ':'))
                params['footprint_padding'] = settings['footprint_padding']
                for key, path in SCALARS.items():
                    parent = params
                    parts = path.split('.')
                    for segment in parts[:-1]:
                        parent = parent[segment]
                    parent[parts[-1]] = settings[name][key]
            updated = yaml.safe_dump(doc, allow_unicode=True, sort_keys=False).encode('utf-8')
            if updated != raw:
                # Replace one complete file so launch readers never see half a YAML.
                tmp = None
                try:
                    with tempfile.NamedTemporaryFile(dir=self.path.parent, prefix='.costmap-', delete=False) as f:
                        tmp = Path(f.name)
                        f.write(updated)
                        f.flush()
                        os.fsync(f.fileno())
                    os.chmod(tmp, self.path.stat().st_mode & 0o777)
                    self.path.with_suffix('.yaml.previous').write_bytes(raw)
                    os.replace(tmp, self.path)
                except OSError as exc:
                    raise SettingsError('配置保存失败：' + str(exc), 503) from exc
                finally:
                    if tmp is not None and tmp.exists():
                        tmp.unlink()
            return self.read()


def differences(saved, runtime):
    """Compare only fresh observed parameters; unknown never means applied."""
    output = []
    for name in ('global', 'local'):
        row = runtime.get(name, {})
        if not row.get('available'):
            return None
        expected = {**saved[name], 'footprint': saved['footprint'], 'footprint_padding': saved['footprint_padding']}
        actual = row['values']
        for key, value in expected.items():
            if key == 'footprint':
                match = len(value) == len(actual[key]) and all(
                    len(a) == len(b) and all(math.isclose(x, y, abs_tol=1e-7) for x, y in zip(a, b))
                    for a, b in zip(value, actual[key]))
            else:
                match = math.isclose(value, actual[key], abs_tol=1e-7)
            if not match:
                output.append(name + '.' + key)
    return output


class RuntimeCostmapSettings:
    """Read both parameter services on the already-spinning mapping executor."""
    def __init__(self, node, clock=time.monotonic):
        from rcl_interfaces.srv import GetParameters
        self.request = GetParameters.Request
        self.clock = clock
        self.lock = threading.RLock()
        self.clients = {name: node.create_client(GetParameters, '/' + name + '_costmap/' + name + '_costmap/get_parameters')
                        for name in ('global', 'local')}
        self.rows = {}
        self.pending = {}
        self.sent = {}
        node.create_timer(1., self.poll)

    def poll(self):
        now = self.clock()
        for name, client in self.clients.items():
            with self.lock:
                pending = self.pending.get(name)
                if pending and now - pending[1] <= 3:
                    continue
                if pending:
                    self.pending.pop(name, None)
                    pending[0].cancel()
                if now - self.sent.get(name, -1e9) < 3 or not client.service_is_ready():
                    continue
                request = self.request()
                request.names = PARAMETERS
                future = client.call_async(request)
                self.pending[name] = (future, now)
                self.sent[name] = now
            future.add_done_callback(lambda result, n=name: self._reply(n, result))

    def _reply(self, name, future):
        with self.lock:
            if self.pending.get(name, (None,))[0] is not future:
                return
            self.pending.pop(name, None)
        try:
            response = future.result()
            if len(response.values) != len(PARAMETERS):
                raise ValueError('参数响应不完整')
            values = {}
            for key, value in zip(PARAMETERS, response.values):
                if key == 'footprint':
                    if value.type != 4:
                        raise ValueError('未提供多边形车体轮廓')
                    points = json.loads(value.string_value)
                    if not isinstance(points, list) or not all(isinstance(p, list) and len(p) == 2 and
                        all(isinstance(v, (int, float)) and math.isfinite(v) for v in p) for p in points):
                        raise ValueError('运行中的车体轮廓无效')
                    values[key] = points
                else:
                    if value.type not in (2, 3):
                        raise ValueError(key + ' 未声明')
                    number = value.double_value if value.type == 3 else float(value.integer_value)
                    if not math.isfinite(number):
                        raise ValueError(key + ' 不是有限值')
                    values[key] = number
            values = {'footprint': values['footprint'], 'footprint_padding': values['footprint_padding'],
                      **{key: values[path] for key, path in SCALARS.items()}}
            row = {'values': values, 'received': self.clock()}
        except Exception as exc:
            row = {'error': str(exc)}
        with self.lock:
            self.rows[name] = row

    def snapshot(self):
        with self.lock:
            result = {}
            for name in ('global', 'local'):
                row = self.rows.get(name, {})
                age = self.clock() - row.get('received', -1e9)
                available = bool(row.get('values')) and age <= 8 and self.clients[name].service_is_ready()
                result[name] = {'available': available}
                if available:
                    result[name].update(values=copy.deepcopy(row['values']), age_sec=round(age, 2))
                else:
                    result[name]['issue'] = row.get('error', '等待运行中的 Nav2 参数，或参数已过期')
            return result
