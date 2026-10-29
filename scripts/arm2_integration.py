#!/usr/bin/env python3
"""Mock-only end-to-end test with REAL MoveIt and protocol adapter; no serial port."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import threading
import time
import uuid
import yaml
import rclpy
from ament_index_python.packages import get_package_share_directory
from dual_arm_interfaces.srv import ConfirmStep
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--scenario', choices=['eight', 'cancel', 'release_failure', 'recovery', 'verify_failure',
        'estop', 'safety_loss', 'stale_safety', 'blocked_b', 'count_failure', 'collision', 'wrong_slot', 'grid_failure', 'grid_mismatch'], default='eight')
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '164':
        raise SystemExit('Use scripts/test_arm2_integration.sh (isolated domain 164)')
    out = Path('/tmp/dual-arm-ros2/arm2-integration') / (args.scenario + '-' + uuid.uuid4().hex[:8])
    out.mkdir(parents=True)
    cfg = yaml.safe_load((Path(get_package_share_directory('dual_arm_control')) / 'config/arm2_demo.yaml').read_text())
    cfg['cm530_bridge']['ros__parameters']['mock_time_scale'] = 0.15
    task = cfg['arm2_coordinator']['ros__parameters']
    task['home_settle'] = 0.15
    task['zone_wait_timeout'] = 1.5
    task['manual_timeout'] = 15.0
    if args.scenario == 'collision':
        p = task['b_pick']
        task['obstacle_boxes'] = p[:3] + [0.08, 0.08, 0.12]
    cfg['arm2_mock_vision'] = {'ros__parameters': {
        'block_b': args.scenario == 'blocked_b', 'fail_count': args.scenario == 'count_failure',
        'freeze_safety_on_grasp': args.scenario == 'stale_safety',
        'wrong_slot': args.scenario == 'wrong_slot', 'fail_grid': args.scenario == 'grid_failure',
        'grid_mismatch': args.scenario == 'grid_mismatch'}}
    config_path = out / 'config.yaml'
    config_path.write_text(yaml.safe_dump(cfg))
    log = (out / 'launch.log').open('w')
    command = ['ros2', 'launch', 'dual_arm_control', 'arm2.launch.py', 'mode:=demo',
               f'config:={config_path}', f'journal:={out}/journal.json']
    proc = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init()
    node = Node('arm2_integration_test')
    state, history, confirmed = {}, [], set()
    lock = threading.Lock()
    def status(msg):
        value = json.loads(msg.data)
        with lock:
            state.clear()
            state.update(value)
            key = [value['cycle_id'], value['state']]
            if not history or history[-1] != key:
                history.append(key)
    node.create_subscription(String, '/arm2/status', status,
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    start = node.create_client(Trigger, '/arm2/b_ready')
    stop = node.create_client(Trigger, '/arm2/cancel')
    confirm = node.create_client(ConfirmStep, '/arm2/confirm')
    estop = node.create_publisher(Bool, '/safety/estop', 10)
    arm1_clear = node.create_publisher(Bool, '/handoff/arm1_clear', 10)
    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()
    def call(client, request, timeout=8.):
        assert client.wait_for_service(timeout_sec=timeout), client.srv_name
        f = client.call_async(request)
        deadline = time.monotonic() + timeout
        while not f.done() and time.monotonic() < deadline:
            time.sleep(.01)
        if not f.done(): raise TimeoutError(client.srv_name)
        return f.result()
    def snapshot():
        with lock: return dict(state)
    def shutdown_launch():
        if proc.poll() is None:
            os.killpg(proc.pid, signal.SIGINT)
            try: proc.wait(timeout=12)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait(timeout=5)
    success = False
    try:
        deadline = time.monotonic() + 40
        while not start.wait_for_service(timeout_sec=.5):
            if proc.poll() is not None or time.monotonic() > deadline:
                raise RuntimeError('Bringup failed')
        time.sleep(3)
        for cycle in range(8 if args.scenario == 'eight' else 1):
            old_cycle = snapshot().get('cycle_id')
            res = call(start, Trigger.Request())
            assert res.success, res.message
            assert not call(start, Trigger.Request()).success, 'Duplicate B_READY accepted'
            deadline = time.monotonic() + 180
            while time.monotonic() < deadline:
                if proc.poll() is not None: raise RuntimeError('Launch exited')
                s = snapshot()
                if s.get('cycle_id') != old_cycle and s.get('state') in ('SUCCEEDED','FULL','FAULT') and not s.get('busy', True):
                    break
                token, op = s.get('confirmation_token'), s.get('confirmation_operation')
                if token and token not in confirmed and op in ('verify_layout','grasp','release','verify_slot'):
                    assert not call(confirm, ConfirmStep.Request(token='stale', success=True)).accepted
                    if args.scenario == 'cancel' and op == 'grasp':
                        assert call(stop, Trigger.Request()).success
                    elif args.scenario == 'estop' and op == 'grasp':
                        estop.publish(Bool(data=True))
                    elif args.scenario == 'safety_loss' and op == 'grasp':
                        arm1_clear.publish(Bool(data=False))
                    elif args.scenario == 'stale_safety' and op == 'grasp':
                        pass  # Let both heartbeat leases expire; do not confirm grasp.
                    else:
                        ok = not ((args.scenario in ('release_failure', 'recovery') and op == 'release') or
                                  (args.scenario == 'verify_failure' and op == 'verify_slot'))
                        res = call(confirm, ConfirmStep.Request(token=token, success=ok, message='mock test operator'))
                        assert res.accepted, res.message
                        assert not call(confirm, ConfirmStep.Request(token=token, success=True)).accepted
                    confirmed.add(token)
                time.sleep(.02)
            else: raise TimeoutError('Cycle did not terminate')
            final = snapshot()
            if args.scenario == 'eight':
                assert final['state'] == ('FULL' if cycle == 7 else 'SUCCEEDED'), final
                assert final['occupied'] == [True] * (cycle + 1) + [False] * (7 - cycle)
                assert not final['b_locked'] and final['holding'] == 'empty'
                print(f'PASS slot {cycle + 1}', flush=True)
            else:
                assert final['state'] == 'FAULT' and final['occupied'] == [False] * 8, final
                if args.scenario in ('cancel', 'estop', 'safety_loss', 'stale_safety'):
                    assert final['holding'] == 'unknown' and final['b_locked'], final
                assert not call(start, Trigger.Request()).success
        if args.scenario == 'eight':
            assert len(confirmed) == 32
            assert not call(start, Trigger.Request()).success, 'Ninth load must be refused'
        # Restart with the SAME journal: faults and occupied slots must survive.
        shutdown_launch()
        with lock: state.clear()
        proc = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        deadline = time.monotonic() + 40
        while time.monotonic() < deadline:
            restored = snapshot()
            if restored and not restored.get('busy', True): break
            time.sleep(.1)
        else: raise TimeoutError('Restart did not publish status')
        assert restored['occupied'] == final['occupied'], restored
        assert restored['state'] == final['state'], restored
        assert restored['confirmation_operation'] == 'recovery' and restored['confirmation_token']
        assert not call(start, Trigger.Request()).success, 'Restart must not clear FULL/FAULT'
        recovered = None
        if args.scenario == 'recovery':
            time.sleep(1.0)  # New mock safety publishers and joint state must be live.
            clear = node.create_client(Trigger, '/arm2_mock/clear_table')
            recover = node.create_client(ConfirmStep, '/arm2/recover')
            assert call(clear, Trigger.Request()).success
            assert not call(recover, ConfirmStep.Request(token='old-token', success=True)).accepted
            res = call(recover, ConfirmStep.Request(token=restored['confirmation_token'], success=True,
                message='mock operator: cleared tool/B/C and restored Home_C'))
            assert res.accepted, res.message
            deadline = time.monotonic() + 5
            while time.monotonic() < deadline:
                recovered = snapshot()
                if recovered.get('state') == 'IDLE' and not recovered.get('busy', True): break
                time.sleep(.05)
            assert recovered['state'] == 'IDLE' and recovered['occupied'] == [False] * 8
            assert not recovered['b_locked'] and recovered['holding'] == 'empty'
        report = dict(passed=True, recovered=recovered, scenario=args.scenario, final=final, restored=restored,
                      states=history, real_moveit=True, actuator='mock', vision='mock', hardware_tested=False)
        (out / 'result.json').write_text(json.dumps(report, indent=2))
        success = True
        print(json.dumps({'passed': True, 'report': str(out / 'result.json')}), flush=True)
    finally:
        if not success: print(f'FAILED: inspect {out}/launch.log; status={snapshot()}', flush=True)
        shutdown_launch()
        rclpy.shutdown()
        spinner.join(timeout=2)
        node.destroy_node()
        log.close()


if __name__ == '__main__': main()
