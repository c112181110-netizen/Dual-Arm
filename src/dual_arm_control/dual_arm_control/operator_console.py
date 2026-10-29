"""Human-only confirmation UI. Never writes to ESP32 or opens a serial port."""
import json
import threading
import rclpy
from dual_arm_interfaces.srv import ConfirmStep
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from .moveit_client import wait_future


def main(args=None):
    rclpy.init(args=args)
    node = Node('arm1_operator_console')
    arm = node.declare_parameter('arm', 'arm1').value
    if arm not in ('arm1', 'arm2'):
        raise ValueError('arm must be arm1 or arm2')
    lock, changed = threading.Lock(), threading.Event()
    current = {}
    def receive(msg):
        with lock:
            current.clear()
            current.update(json.loads(msg.data))
        changed.set()
    node.create_subscription(String, f'/{arm}/status', receive,
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    confirm = node.create_client(ConfirmStep, f'/{arm}/confirm')
    recover = node.create_client(ConfirmStep, f'/{arm}/recover')
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()
    seen = ''
    try:
        print('等待人工操作點。此工具不控制 ESP32；請先在外部操作吸盤。')
        while rclpy.ok():
            changed.wait(0.5)
            changed.clear()
            with lock:
                state = dict(current)
            token = state.get('confirmation_token', '')
            if not token or token == seen:
                continue
            seen = token
            operation = state['confirmation_operation']
            prompt = {'grasp': '請開啟吸盤，確認圓片已吸住', 'release': '請釋放吸盤，確認圓片已落在 B 區',
                      'recovery': '請先停止並處理異常，確認工具無物件、B 區淨空且手臂已回 Home_A'}.get(operation, operation)
            if arm == 'arm2':
                prompt = {'grasp': '請開啟吸盤並確認物件已吸住',
                          'release': f'請關閉吸盤並確認物件已釋放到 C 第 {state.get("slot")} 格',
                          'verify_layout': f'核對 C 全部格位與紀錄 {state.get("occupied")} 一致（true=占用），且第 {state.get("slot")} 格空、路徑無障礙',
                          'verify_slot': f'確認物件穩定落在 C 第 {state.get("slot")} 格內，工具無物件',
                          'recovery': '先處理現場：清空工具與 B/C 八格、手臂二回 Home_C、手臂一固定在安全區；這會清除八格紀錄'}.get(operation, operation)
            answer = input(f'\n{prompt}。輸入 yes 確認、no 回報失敗，其他輸入略過：').strip().lower()
            if answer not in ('yes', 'no'):
                seen = ''
                continue
            client = recover if operation == 'recovery' else confirm
            if not client.wait_for_service(timeout_sec=2.0):
                print('服務不可用')
                continue
            response = wait_future(client.call_async(ConfirmStep.Request(
                token=token, success=answer == 'yes', message='operator console')), 5.0)
            print(response.message)
            if not response.accepted:
                seen = ''
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        thread.join(timeout=2.0)
        node.destroy_node()
