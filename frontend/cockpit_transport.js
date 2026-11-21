/* Transport for the real robot. No DOM, simulation or ROS motion publishing. */
(function (global) {
  'use strict';
  class CockpitTransport {
    constructor(callbacks) {
      this.cb = callbacks;
      this.epoch = 0;
      this.sockets = [];
      this.command = { linear_x: 0, linear_y: 0, angular_z: 0 };
      this.connected = { ros: false, agent: false, control: false };
      this.timer = setInterval(() => this.tick(), 100);
    }
    status() { this.cb.status(Object.assign({}, this.connected)); }
    zero() {
      const moving = !!(this.command.linear_x || this.command.linear_y || this.command.angular_z);
      this.command = { linear_x: 0, linear_y: 0, angular_z: 0 };
      // Idle telemetry/estop must not generate a zero-command rejection every second.
      if (moving && this.controlSocket && this.controlSocket.readyState === 1) {
        this.controlSocket.send(JSON.stringify(this.command));
      }
    }
    close() {
      this.zero();
      this.epoch++;
      clearTimeout(this.retry);
      this.sockets.forEach(s => s.close());
      this.sockets = [];
      this.controlSocket = null;
      this.running = false;
      this.ready = false;
      this.connected = { ros: false, agent: false, control: false };
      this.cb.reset();
      this.status();
    }
    start(config) {
      this.close();
      this.config = config;
      this.running = true;
      this.lastTelemetry = 0;
      this.startedAt = Date.now();
      const epoch = this.epoch;
      const open = (name, port, path) => {
        const ws = new WebSocket('ws://' + config.ip + ':' + port + path);
        this.sockets.push(ws);
        ws.onclose = () => {
          if (epoch !== this.epoch) return;
          this.connected[name] = false;
          this.ready = false;
          this.zero();
          this.cb.reset();
          this.status();
          if (!this.retry) this.retry = setTimeout(() => {
            this.retry = null;
            if (epoch === this.epoch && this.running) this.start(this.config);
          }, 3000);
        };
        ws.onerror = () => {
          if (epoch === this.epoch) this.cb.error(name + ' 连接失败，请检查服务和端口');
        };
        return ws;
      };
      const ros = open('ros', config.rosPort, '');
      ros.onopen = () => {
        if (epoch !== this.epoch) return;
        this.connected.ros = true;
        this.status();
        ros.send(JSON.stringify({ op: 'subscribe', topic: '/odom', type: 'nav_msgs/msg/Odometry',
          throttle_rate: 100, queue_length: 1 }));
      };
      ros.onmessage = e => {
        if (epoch !== this.epoch) return;
        try { this.cb.ros(JSON.parse(e.data)); } catch (err) { this.cb.error(err.message); }
      };
      const agent = open('agent', config.agentPort, '/ws/telemetry');
      agent.onmessage = e => {
        if (epoch !== this.epoch) return;
        try {
          const data = JSON.parse(e.data);
          if (!data.control || !data.capabilities) throw new Error('网关版本不匹配，请同步更新后端');
          this.connected.agent = true;
          this.lastTelemetry = Date.now();
          this.ready = !!data.control_ready && !data.estop_engaged;
          if (!this.ready) { this.zero(); this.cb.reset(); }
          this.cb.telemetry(data);
          this.status();
        } catch (err) {
          this.ready = false;
          this.connected.agent = false;
          this.zero(); this.cb.reset(); this.status();
          this.cb.error(err.message);
        }
      };
      const control = open('control', config.agentPort, '/ws/control');
      this.controlSocket = control;
      control.onmessage = e => {
        if (epoch !== this.epoch) return;
        try {
          const data = JSON.parse(e.data);
          if (data.type === 'ready') this.connected.control = true;
          this.lastAck = Date.now();
          if (data.ok === false) {
            // Do not retry a rejected command automatically (especially ownership conflicts).
            this.command = { linear_x: 0, linear_y: 0, angular_z: 0 };
            this.cb.reset();
            this.cb.error(data.detail || '控制指令被拒绝');
          }
          this.status();
        } catch (err) { this.cb.error(err.message); }
      };
    }
    canControl() {
      return this.running && this.ready && this.connected.agent && this.connected.control &&
        Date.now() - this.lastTelemetry < 3000;
    }
    velocity(x, y, yaw) {
      const moving = !!(x || y || yaw);
      if (moving && !this.canControl()) {
        this.cb.reset();
        throw new Error('控制未就绪：请检查网关、底盘订阅者和急停状态');
      }
      this.command = { linear_x: x, linear_y: y, angular_z: yaw };
      if (this.controlSocket && this.controlSocket.readyState === 1) {
        this.controlSocket.send(JSON.stringify(this.command));
      } else if (moving) throw new Error('控制通道未连接');
    }
    tick() {
      if (!this.running) return;
      if (!this.lastTelemetry && Date.now() - this.startedAt > 8000) {
        this.cb.error('未收到网关遥测，请检查后端版本和 8000 端口');
        this.start(this.config);
        return;
      }
      if (this.connected.agent && Date.now() - this.lastTelemetry >= 3000) {
        this.ready = false;
        this.connected.agent = false;
        this.zero();
        this.cb.reset();
        this.status();
        // A stalled connection may never fire onclose. Force a fresh connection.
        this.start(this.config);
        return;
      }
      if (this.command.linear_x || this.command.linear_y || this.command.angular_z) {
        if (!this.canControl() || Date.now() - this.lastAck > 1000) {
          this.zero();
          this.cb.reset();
          this.cb.error('控制确认超时，已停止续发速度');
          return;
        }
        this.controlSocket.send(JSON.stringify(this.command));
      }
    }
    async api(path, body, timeout = 6000) {
      const config = this.config;
      if (!config || !this.running) throw new Error('尚未连接实车网关');
      const epoch = this.epoch;
      const abort = new AbortController();
      const timer = setTimeout(() => abort.abort(), timeout);
      try {
        const response = await fetch('http://' + config.ip + ':' + config.agentPort + path, {
          method: body === undefined ? 'GET' : 'POST',
          headers: body === undefined ? {} : { 'Content-Type': 'application/json' },
          body: body === undefined ? undefined : JSON.stringify(body), signal: abort.signal,
        });
        const data = await response.json();
        if (epoch !== this.epoch) throw new Error('连接配置已改变，请重新操作');
        if (!response.ok || data.ok === false) throw new Error(data.detail || data.stderr || ('HTTP ' + response.status));
        return data;
      } finally { clearTimeout(timer); }
    }
  }
  global.CockpitTransport = CockpitTransport;
})(typeof window === 'undefined' ? globalThis : window);
