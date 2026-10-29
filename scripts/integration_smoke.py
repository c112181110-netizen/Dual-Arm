#!/usr/bin/env python3
"""Real MoveIt + model + mock CM530/vision. Never launches hardware mode.

Run after sourcing the built workspace. A private ROS domain is required.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import threading
import time
import uuid

import rclpy
from dual_arm_interfaces.srv import ConfirmStep
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--scenario', choices=['full', 'cancel', 'release_failure'], default='full')
    parser.add_argument('--output-dir', default='/tmp/dual-arm-ros2/integration')
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '163':
        raise SystemExit('Use ROS_DOMAIN_ID=163 for this isolated mock-only test')
    out = Path(args.output_dir) / (args.scenario + '-' + uuid.uuid4().hex[:8])
    out.mkdir(parents=True)
    log = (out / 'launch.log').open('w')
    proc = subprocess.Popen(['ros2', 'launch', 'dual_arm_control', 'arm1.launch.py',
                             'mode:=demo', 'auto_confirm_demo:=false', f'journal:={out}/journal.json'],
                            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init()
    node = Node('arm1_integration_test')
    state, history, confirmed = {}, [], set()
    lock = threading.Lock()
    def status(msg):
        value = json.loads(msg.data)
        with lock:
            state.clear(); state.update(value)
            if not history or history[-1] != value['state']:
                history.append(value['state'])
    node.create_subscription(String, '/arm1/status', status,
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    start = node.create_client(Trigger, '/arm1/start')
    stop = node.create_client(Trigger, '/arm1/cancel')
    confirm = node.create_client(ConfirmStep, '/arm1/confirm')
    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()
    def call(client, req, timeout=8.0):
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f'Missing service: {client.srv_name}')
        future = client.call_async(req)
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        if not future.done():
            raise TimeoutError(client.srv_name)
        return future.result()
    success = False
    try:
        deadline = time.monotonic() + 40.0
        while not start.wait_for_service(timeout_sec=0.5):
            if proc.poll() is not None or time.monotonic() > deadline:
                raise RuntimeError(f'Bringup failed; inspect {out}/launch.log')
        # Allow move_group to initialize its planning scene and action interfaces.
        time.sleep(3.0)
        response = call(start, Trigger.Request())
        assert response.success, response.message
        deadline = time.monotonic() + 240.0
        while time.monotonic() < deadline:
            if proc.poll() is not None:
                raise RuntimeError('Bringup exited unexpectedly')
            with lock:
                snapshot = dict(state)
            if snapshot.get('state') in ('SUCCEEDED', 'FAULT') and not snapshot.get('busy', True):
                break
            token = snapshot.get('confirmation_token')
            op = snapshot.get('confirmation_operation')
            if token and op in ('grasp', 'release') and token not in confirmed:
                # Stale confirmations must not advance the workflow.
                bad = call(confirm, ConfirmStep.Request(token='stale-token', success=True))
                assert not bad.accepted
                time.sleep(0.3)
                with lock:
                    assert state.get('confirmation_token') == token, 'Manual wait advanced without confirmation'
                if args.scenario == 'cancel' and op == 'grasp':
                    assert call(stop, Trigger.Request()).success
                else:
                    accepted = call(confirm, ConfirmStep.Request(
                        token=token, success=not (args.scenario == 'release_failure' and op == 'release'),
                        message='integration test operator'))
                    assert accepted.accepted, accepted.message
                    duplicate = call(confirm, ConfirmStep.Request(token=token, success=True))
                    assert not duplicate.accepted
                confirmed.add(token)
            time.sleep(0.05)
        else:
            raise TimeoutError('Workflow did not terminate')
        with lock:
            final = dict(state)
        if args.scenario == 'full':
            assert final['state'] == 'SUCCEEDED', final
            assert not final['b_locked'] and final['holding'] == 'empty'
            assert len(confirmed) == 2
            for step in ['ARM1_STATE1_WAIT_SUCTION_ON', 'ARM1_STATE2_WAIT_SUCTION_OFF', 'ARM1_STATE3_VERIFY_B']:
                assert step in history
        else:
            assert final['state'] == 'FAULT' and final['holding'] == 'unknown', final
            assert final['b_locked'] == (args.scenario == 'release_failure')
            assert not call(start, Trigger.Request()).success, 'Fault must block automatic restart'
        success = True
        report = {'scenario': args.scenario, 'passed': True, 'states': history, 'final': final,
                  'real_moveit': True, 'actuator': 'mock', 'vision': 'mock', 'esp32_connected': False}
        (out / 'result.json').write_text(json.dumps(report, indent=2))
        print(json.dumps({'passed': True, 'scenario': args.scenario, 'report': str(out / 'result.json')}), flush=True)
    finally:
        if not success:
            print(f'FAILED: inspect {out}/launch.log', flush=True)
        if proc.poll() is None:
            os.killpg(proc.pid, signal.SIGINT)
            try:
                proc.wait(timeout=12)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait(timeout=5)
        rclpy.shutdown()
        spinner.join(timeout=2)
        node.destroy_node()
        log.close()


if __name__ == '__main__':
    main()
