import rclpy                       # 导入 ROS 2 的 Python 客户端核心库（必须项）
from rclpy.node import Node        # 导入 Node 基类，ROS 2 中的所有节点都必须继承它
import time                        # 导入 Python 标准时间库

class HelloWorldNode(Node):
    def __init__(self, name):      
        # 调用父类（Node）的初始化方法，并在 ROS 2 网络中为这个节点“注册”一个唯一的名字
        super().__init__(name)                         
        
        # 主循环：rclpy.ok() 会检查 ROS 2 系统是否正常（比如有没有按下 Ctrl+C 终止程序）
        while rclpy.ok():                              
            # get_logger().info() 是 ROS 2 专属的打印方式。
            self.get_logger().info("Hello World")       
            time.sleep(0.5)        # 暂停 0.5 秒，控制循环频率

def main(args=None):
    # 第一步：初始化 ROS 2 通信系统（万物之源，运行任何 ROS 2 节点前必须调用）
    rclpy.init(args=args)                              
    
    # 第二步：实例化我们上面写的类。
    node = HelloWorldNode("node_helloworld_class")     
                                                       
    # 第三步：让节点“转”起来（Spin），保持节点处于运行状态。
    rclpy.spin(node)                                   
    
    # 第四步：收尾工作。
    node.destroy_node()                                
    rclpy.shutdown()

if __name__ == "__main__":
    main()