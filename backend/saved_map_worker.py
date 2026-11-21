"""Run one uniquely named map_server and activate only that owned lifecycle node.

This process has no velocity publishers, localization, navigation or driver launches.
"""
import argparse
import json
import signal
import subprocess
import sys
import time


def main():
    import rclpy
    from lifecycle_msgs.msg import Transition
    from lifecycle_msgs.srv import ChangeState
    parser=argparse.ArgumentParser()
    parser.add_argument('--yaml',required=True)
    parser.add_argument('--node-name',required=True)
    args=parser.parse_args()
    rclpy.init(args=None)
    node=rclpy.create_node(args.node_name+'_loader')
    child=None
    try:
        child=subprocess.Popen(['ros2','run','nav2_map_server','map_server','--ros-args',
            '-r','__node:='+args.node_name,'-r','__ns:=/','-r','map:=/map',
            '-p','yaml_filename:='+args.yaml,'-p','use_sim_time:=false'])
        client=node.create_client(ChangeState,'/'+args.node_name+'/change_state')
        deadline=time.monotonic()+6
        while not client.wait_for_service(timeout_sec=0.1):
            if child.poll() is not None: raise RuntimeError('map_server 启动失败')
            if time.monotonic()>deadline: raise RuntimeError('地图服务启动超时')
        for transition in (Transition.TRANSITION_CONFIGURE,Transition.TRANSITION_ACTIVATE):
            request=ChangeState.Request();request.transition.id=transition
            future=client.call_async(request)
            rclpy.spin_until_future_complete(node,future,timeout_sec=3)
            if not future.done():
                future.cancel();raise RuntimeError('地图生命周期切换超时')
            result=future.result()
            if not result or not result.success: raise RuntimeError('地图配置/激活失败，请检查 YAML 与图像文件')
        print('COCKPIT_MAP_READY '+json.dumps({'yaml':args.yaml,'node':args.node_name}),flush=True)
        while child.poll() is None: time.sleep(0.2)
        return child.returncode
    except KeyboardInterrupt:
        return 0
    except Exception as exc:
        print('COCKPIT_MAP_ERROR '+str(exc),flush=True)
        return 1
    finally:
        if child and child.poll() is None:
            child.send_signal(signal.SIGINT)
            try:child.wait(timeout=2)
            except subprocess.TimeoutExpired:child.terminate()
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':sys.exit(main())
