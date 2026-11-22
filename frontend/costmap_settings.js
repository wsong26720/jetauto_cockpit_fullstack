/* Saved configuration and observed ROS parameters are deliberately separate. */
(function (global) {
  'use strict';
  const fields = [
    ['inflation_radius', '膨胀半径', 'm', 0.1, 2, 0.01],
    ['cost_scaling_factor', '代价衰减系数', '', 0.1, 30, 0.1],
    ['resolution', '配置分辨率', 'm/格', 0.02, 0.1, 0.005],
    ['obstacle_max_range', '障碍标记距离', 'm', 0.2, 20, 0.1],
    ['raytrace_max_range', '清障距离', 'm', 0.2, 25, 0.1],
  ];
  const el = id => document.getElementById(id);
  const clone = value => JSON.parse(JSON.stringify(value));

  class CostmapSettingsPanel {
    constructor(api) {
      this.api = api;
      this.connected = false;
      this.epoch = 0;
      this.dirty = false;
      this.pending = false;
      el('costmapSettingsRows').innerHTML = fields.map(([key, title, unit, min, max, step]) =>
        '<tr><th scope="row" class="py-2 pr-2 text-left font-medium">' + title +
        (unit ? '<span class="text-slate-400"> (' + unit + ')</span>' : '') + '</th>' +
        ['global', 'local'].map(name => '<td class="py-2 pl-2"><input id="cm-' + name + '-' + key +
          '" aria-label="' + (name === 'global' ? '全局' : '局部') + title +
          '" type="number" required min="' + min + '" max="' + max + '" step="' + step +
          '" class="w-full rounded border border-slate-300 px-2 py-1 bg-white">' +
          '<div id="cm-live-' + name + '-' + key + '" class="mt-1 text-[10px] text-slate-500">运行：未知</div></td>').join('') + '</tr>'
      ).join('');
      el('costmapSettingsForm').addEventListener('input', () => {
        this.dirty = true;
        this.updateButtons();
        this.preview();
      });
      el('costmapSettingsForm').addEventListener('submit', event => { event.preventDefault(); this.save(); });
      el('btnReadCostmapSettings').addEventListener('click', () => this.read(true));
      el('btnResetCostmapSettings').addEventListener('click', () => {
        if (this.report) { this.fill(this.report.settings); this.dirty = false; this.updateButtons(); this.status('已恢复上次读取值，尚未保存'); }
      });
      el('costmapSettingsPanel').addEventListener('toggle', () => {
        if (el('costmapSettingsPanel').open && this.connected && !this.pending) this.read(false);
      });
      this.timer = setInterval(() => {
        if (this.connected && !this.pending && el('costmapSettingsPanel').open) this.read(false);
      }, 8000);
      this.updateButtons();
    }

    status(message, error = false) {
      const target = el('costmapSettingsStatus');
      target.textContent = message;
      target.className = 'mt-2 text-xs ' + (error ? 'text-red-700' : 'text-slate-600');
      target.setAttribute('role', error ? 'alert' : 'status');
    }

    setConnected(connected) {
      if (this.connected === connected) return;
      this.connected = connected;
      this.epoch++;
      this.report = null;
      this.pending = false;
      this.dirty = false;
      this.updateButtons();
      if (connected) this.read(true);
      else {
        document.querySelectorAll('[id^="cm-live-"]').forEach(node => { node.textContent = '运行：未知'; });
        el('costmapRuntimeFootprint').textContent = '运行车体轮廓：未知';
        el('costmapSettingsWarnings').textContent = '';
        this.status('未连接实车网关，配置不可保存');
      }
    }

    updateButtons() {
      el('btnReadCostmapSettings').disabled = !this.connected || this.pending;
      el('btnReadCostmapSettings').textContent = this.dirty ? '重新读取（丢弃输入）' : '读取配置';
      el('btnSaveCostmapSettings').disabled = !this.connected || this.pending || !this.report || !this.dirty;
      el('btnSaveCostmapSettings').textContent = this.pending ? '处理中…' : '保存，下次启动生效';
      el('btnResetCostmapSettings').disabled = !this.report || this.pending || !this.dirty;
      el('costmapSettingsInputs').disabled = !this.connected || !this.report || (this.pending && !this.refreshOnly);
    }

    fill(settings) {
      el('cm-footprint').value = JSON.stringify(settings.footprint);
      el('cm-padding').value = settings.footprint_padding;
      for (const name of ['global', 'local']) for (const [key] of fields) el('cm-' + name + '-' + key).value = settings[name][key];
      this.preview();
    }

    preview() {
      try {
        const p = JSON.parse(el('cm-footprint').value);
        const padding = el('cm-padding').valueAsNumber;
        if (!Array.isArray(p) || p.length < 3 || !Number.isFinite(padding) || !p.every(v => Array.isArray(v) && v.length === 2 && v.every(Number.isFinite))) throw Error();
        const width = Math.max(...p.map(v => v[1])) - Math.min(...p.map(v => v[1]));
        const length = Math.max(...p.map(v => v[0])) - Math.min(...p.map(v => v[0]));
        el('costmapFootprintPreview').textContent = '轮廓包围盒：长 ' + length.toFixed(3) + ' × 宽 ' + width.toFixed(3) +
          ' m；矩形加余量后约 ' + (length + 2 * padding).toFixed(3) + ' × ' + (width + 2 * padding).toFixed(3) + ' m';
      } catch (_) { el('costmapFootprintPreview').textContent = '请输入有效轮廓 JSON 和余量'; }
    }

    showRuntime(report) {
      const profiles = [];
      for (const name of ['global', 'local']) {
        const row = report.runtime?.[name];
        const label = name === 'global' ? '全局' : '局部';
        for (const [key] of fields) {
          el('cm-live-' + name + '-' + key).textContent = row?.available ? '运行：' + row.values[key] : '运行：未读取';
        }
        if (row?.available) profiles.push(label + '：余量 ' + row.values.footprint_padding + ' m，轮廓 ' + JSON.stringify(row.values.footprint));
        else profiles.push(label + '：' + (row?.issue || '运行参数未读取'));
        if (Number.isFinite(row?.grid_resolution)) el('cm-live-' + name + '-resolution').textContent += '；实际栅格 ' + row.grid_resolution.toFixed(3);
      }
      el('costmapRuntimeFootprint').textContent = '运行车体轮廓｜' + profiles.join('；');
      el('costmapSettingsWarnings').textContent = (report.warnings || []).join('；');
    }

    async read(overwrite) {
      if (!this.connected || this.pending) return;
      const epoch = this.epoch;
      this.refreshOnly = !overwrite && !!this.report;
      this.pending = true;
      this.updateButtons();
      try {
        const report = await this.api('/api/navigation/costmap-settings');
        if (epoch !== this.epoch) return;
        const changed = this.report && report.revision !== this.report.revision;
        this.showRuntime(report);
        if (overwrite || (!this.dirty && !this.report) || (!this.dirty && changed)) {
          this.report = report;
          this.fill(report.settings);
          this.dirty = false;
        } else if (!this.dirty) this.report = report;
        if (changed && this.dirty && !overwrite) this.status('配置文件已变化；当前输入已保留，请重新读取后再保存', true);
        else if (this.dirty) this.status('有未保存输入；运行参数已刷新');
        else this.status(this.summary(report));
      } catch (error) {
        if (epoch === this.epoch) {
          this.showRuntime({runtime: {}, warnings: []});
          this.status('读取失败：' + error.message, true);
        }
      } finally {
        if (epoch === this.epoch) { this.pending = false; this.updateButtons(); }
      }
    }

    summary(report) {
      if (report.restart_required === null) return '已读取保存配置；运行参数未知，无法确认是否生效。保存后需手动重启导航';
      if (report.restart_required) return '保存配置与运行参数有 ' + report.pending_changes.length + ' 项不同；停稳后手动重启导航模块生效';
      return '保存配置与已读取的运行参数一致；实际栅格分辨率另见下方';
    }

    async save() {
      if (!this.connected || this.pending || !this.report || !this.dirty) return;
      if (!el('costmapSettingsForm').reportValidity()) return;
      let settings;
      try {
        settings = {footprint: JSON.parse(el('cm-footprint').value), footprint_padding: el('cm-padding').valueAsNumber};
        for (const name of ['global', 'local']) {
          settings[name] = {};
          for (const [key] of fields) settings[name][key] = el('cm-' + name + '-' + key).valueAsNumber;
        }
        if (!Number.isFinite(settings.footprint_padding) || ['global', 'local'].some(name => Object.values(settings[name]).some(v => !Number.isFinite(v)))) throw Error('参数必须是有限数字');
      } catch (error) { this.status('输入无效：' + error.message, true); return; }
      const epoch = this.epoch;
      const body = {revision: this.report.revision, settings: clone(settings)};
      this.refreshOnly = false;
      this.pending = true;
      this.updateButtons();
      try {
        const report = await this.api('/api/navigation/costmap-settings', body);
        if (epoch !== this.epoch) return;
        this.report = report;
        this.dirty = false;
        this.fill(report.settings);
        this.showRuntime(report);
        this.status('已保存。' + this.summary(report) + '；未自动重启导航');
      } catch (error) {
        if (epoch === this.epoch) this.status('保存失败：' + error.message + '。输入已保留', true);
      } finally {
        if (epoch === this.epoch) { this.pending = false; this.updateButtons(); }
      }
    }
  }
  global.CostmapSettingsPanel = CostmapSettingsPanel;
})(window);
