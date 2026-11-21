"""Saved-map process ownership and catalog. Never starts robot drivers or goals."""
import json
import os
from pathlib import Path
import re
import shlex
import signal
import subprocess
import threading
import uuid
from fastapi import HTTPException


class SavedMapManager:
    def __init__(self, directory, env_prefix, node_getter):
        self.directory=Path(directory).expanduser().resolve()
        self.env_prefix=env_prefix
        self.node_getter=node_getter
        self.lock=threading.RLock()
        self.proc=None;self.node_name=None;self.name=None
        self.active=False;self.busy=False;self.error=None;self.logs=[]
        self.ready=threading.Event()

    def path(self,name):
        if not re.fullmatch(r'[A-Za-z0-9_-]+',name):
            raise HTTPException(422,'地图名只能包含字母、数字、下划线和连字符')
        path=(self.directory/(name+'.yaml')).resolve()
        if self.directory not in path.parents or not path.is_file():
            raise HTTPException(404,'找不到地图：'+name+'.yaml')
        return path

    def catalog(self):
        maps=[]
        if self.directory.is_dir():
            for path in sorted(self.directory.glob('*.yaml')):
                try:
                    resolved=self.path(path.stem)
                    maps.append({'name':path.stem,'yaml':str(resolved)})
                except HTTPException:pass
        return {'maps':maps,'directory':str(self.directory),'saved_map':self.status()}

    def status(self):
        with self.lock:
            running=self.proc is not None and self.proc.poll() is None
            error=self.error
            if self.proc is not None and not running and self.active:
                error=error or '保存地图服务已退出，退出码 '+str(self.proc.poll())
            return {'name':self.name,'node':self.node_name,'running':running,'active':running and self.active,
                    'busy':self.busy,'error':error,'recent_logs':list(self.logs[-12:])}

    def _read(self,proc,ready):
        for raw in iter(proc.stdout.readline,b''):
            line=raw.decode('utf-8','replace').rstrip()
            with self.lock:
                if proc is not self.proc:continue
                self.logs.append(line[:4000]);del self.logs[:-100]
                if line.startswith('COCKPIT_MAP_READY '):
                    try:payload=json.loads(line[len('COCKPIT_MAP_READY '):])
                    except ValueError:continue
                    if payload.get('node')==self.node_name:
                        self.active=True;ready.set()
                elif line.startswith('COCKPIT_MAP_ERROR '):
                    self.error=line[len('COCKPIT_MAP_ERROR '):];ready.set()
        ready.set()

    @staticmethod
    def _stop_process(proc):
        if proc is None:return
        # start_new_session=True gives this worker and its map_server one owned group.
        try:os.killpg(proc.pid,signal.SIGINT)
        except ProcessLookupError:return
        try:proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid,signal.SIGTERM)
            try:proc.wait(timeout=2)
            except subprocess.TimeoutExpired as exc:
                raise HTTPException(504,'受管地图进程未退出，请查看日志') from exc

    def load(self,name):
        path=self.path(name)
        with self.lock:
            if self.busy:raise HTTPException(409,'已有地图加载/停止请求正在处理')
            node=self.node_getter()
            if node is None:raise HTTPException(503,'ROS 网关尚未初始化')
            others=[p.node_name for p in node.get_publishers_info_by_topic('/map') if p.node_name!=self.node_name]
            if others:raise HTTPException(409,'/map 已由 '+', '.join(sorted(set(others)))+' 发布；请先停止对应 SLAM/地图服务')
            self.busy=True
            previous=self.proc
        proc=None
        try:
            self._stop_process(previous)
            node_name='cockpit_saved_map_'+uuid.uuid4().hex[:10]
            worker=Path(__file__).with_name('saved_map_worker.py')
            args=['python3',str(worker),'--yaml',str(path),'--node-name',node_name]
            command=self.env_prefix+' && exec '+shlex.join(args)
            ready=threading.Event()
            with self.lock:
                self.active=False;self.error=None;self.logs=[];self.name=name;self.node_name=node_name;self.ready=ready
                proc=subprocess.Popen(['/bin/bash','-c',command],stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,start_new_session=True)
                self.proc=proc
            threading.Thread(target=self._read,args=(proc,ready),daemon=True).start()
            if not ready.wait(timeout=14):raise HTTPException(504,'地图加载超时，请查看地图服务日志')
            with self.lock:
                if not self.active or proc.poll() is not None:
                    raise HTTPException(503,self.error or '地图服务未激活：'+'\n'.join(self.logs[-5:]))
            return {'ok':True,'name':name,'yaml':str(path),'node':node_name,'state':'active'}
        except Exception as exc:
            if proc is not None:self._stop_process(proc)
            with self.lock:
                self.active=False
                self.error=str(exc.detail if isinstance(exc,HTTPException) else exc)
            if isinstance(exc,HTTPException):raise
            raise HTTPException(503,str(exc)) from exc
        finally:
            with self.lock:self.busy=False

    def stop(self):
        with self.lock:
            if self.busy:raise HTTPException(409,'地图请求正在处理')
            self.busy=True;proc=self.proc
        try:
            self._stop_process(proc)
            with self.lock:self.proc=None;self.active=False;self.error=None
            return {'ok':True,'state':'stopped'}
        finally:
            with self.lock:self.busy=False
