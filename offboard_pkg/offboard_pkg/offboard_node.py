import rclpy
import rclpy.logging
from rclpy.node import Node
from mavros_msgs.msg import State
# 全局变量
current_state = State()

# 无人机状态回调函数
def state_callback(msg):
    global current_state
    current_state = msg
    #测试打印
    logger = rclpy.logging.get_logger('state_sub')
    logger.info('State: isconnected={} (bool), armed={} (bool)'.format(
        current_state.connected,
        current_state.armed))
    logger.info('State: isguided={} (bool), mode={} (string)'.format(
        current_state.guided,
        current_state.mode))

def main(args=None):
    rclpy.init(args=args)
    node = Node('offboard_node')
    # 创建订阅者
    state_sub = node.create_subscription(State, '/mavros/state', state_callback, 10)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

