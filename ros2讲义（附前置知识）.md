# 0.学习资料

https://fishros.com/d2lros2/#/  （B站讲ros2的非常好的视频）

[往届\_机器狗初赛培训.pptx](files/ros2讲义（附前置知识）-往届_机器狗初赛培训.pptx)

（往届培训PPT和智能狗课程的PPT，该讲义只讲了前三部分，3只讲了一半）

[No1\_机器狗架构\&ROS2基本操作.pptx](files/ros2讲义（附前置知识）-No1_机器狗架构\&ROS2基本操作.pptx)

[No2\_节点&话题.pptx](files/ros2讲义（附前置知识）-No2_节点&话题.pptx)

[No3\_服务&接口.pptx](files/ros2讲义（附前置知识）-No3_服务&接口.pptx)

[No4\_视觉&相机.pptx](files/ros2讲义（附前置知识）-No4_视觉&相机.pptx)

[No5\_动作.pptx](files/ros2讲义（附前置知识）-No5_动作.pptx)

如果按照讲义后未成功，在确定顺序没有问题的情况下可以先问问AI尝试解决、或查看答疑文档

如果没解决的话可以在选手群提问，或者加讲师微信：a19870561520

# 1.前置知识

## 1.1手机遥控

可以先用手机遥控玩一玩，还可以用手机查看电量

安卓手机/平板下载并安装app：**Cyberdog2（只能用安卓手机（安卓人win））**

&#x20;下载链接：https://cloud.tsinghua.edu.cn/f/5cf33f9ef7264a43b803/?dl=1

手机连接实验室**局域网WiFi**：Cyberdog，密码：12345678

在狗的后脑勺触摸区（按住三秒），进入配对模式，然后 登录app，连接机器狗，连接成功后可通过手机/平板遥控机器狗

![机器狗图片](images/image(1).png)

## 1.2vscode

狗的版本很低，需要下载版本很低的vscode：

（实测1.111最新版肯定不行）

（实测cursor也不行）

https://cloud.tsinghua.edu.cn/f/a44a5dd30e8b4b0d8f6c/?dl=1（附上1.98版本的vscode）

下载后需要注意的地方：

1.**避免自动更新**，可以用“cltr+，”打开设置，然后搜索update，在这里改为none，避免自动更新

![](images/ros2讲义（附前置知识）-image.png)


2.如果打不开：**修改文件名字（进入C盘自己的user目录，将下述路径下的User文档改名）**

将C:\Users\…\AppData\Roaming\\**Code**

改为C:\Users\…\AppData\Roaming\\**MyCode**


下载vscode后，记得要下载目前必要的插件：

![](images/ros2讲义（附前置知识）-image-1.png)

在VSCode页面左侧“扩展”商店查找**Python和Remote-ssh**插件并安装

（建议搜个Chinese先汉化重启一下，以及下一个“pylance“更好调代码，还有XML、YAML等等）

VScode插件介绍：https://blog.csdn.net/u011262253/article/details/113879997



## 1.3ssh连接&#x20;

#### 核心作用

通过 SSH 可以在本地电脑远程控制 ROS2 机器人 / 开发板（如树莓派、Jetson），无需外接显示器。

![](images/ros2讲义（附前置知识）-image-2.png)

#### 常用命令

```plain&#x20;text
# 连接远程设备（替换为实际IP和用户名），用户名写在了狗身上
ssh 用户名@设备IP  # 例：ssh mi@10.0.0.x(把x换成你狗上的数字，例如6)

#接下来选择 linux、继续、密码：123即可登入
```

![](images/ros2讲义（附前置知识）-image-3.png)

![选择默认保存位置](images/ros2讲义（附前置知识）-image-4.png)

![](images/ros2讲义（附前置知识）-image-5.png)

![密码123](images/ros2讲义（附前置知识）-image-6.png)

## 1.4Linux与Linux命令行

Linux 命令行（Terminal / Shell）是一种**完全脱离鼠标，通过输入纯文本指令来操控操作系统**的方式。因为机器狗（及大多数机器人开发板）为了节省性能，通常不运行桌面图形系统，所以命令行是我们与它沟通的主力桥梁。

不熟悉Linux的人可以通过下面的视频来快速了解linux

**💡 特别提醒：无需本地安装 Linux！** 本次学习**不需要**在个人电脑上安装 Linux 系统或虚拟机。我们刚才通过 SSH 连上的机器狗，其主板本身就是一个完整的 Linux 环境。我们后续所有的 ROS 2 操作，都将直接在这只狗的系统上完成。

### **可以在vscode连上机器狗后，在vscode里面按下“Ctrl+\~”打开机器狗的终端**

**后面的很多命令都是在这个终端上使用的**

常用linux命令

![](images/ros2讲义（附前置知识）-image-7.png)

特别讲解vim指令（进入文件内部）

![](images/ros2讲义（附前置知识）-image-8.png)

![](images/ros2讲义（附前置知识）-image-9.png)



![](images/ros2讲义（附前置知识）-4897eac56cf14135759adaf73e015bff.jpg)



## 1.5 Tightvnc———— 便于显示图形化界面

本次竞赛推荐采用tightVNC搭建狗子的远程桌面，从而在自己的电脑上对狗子进行可视的编程。

![](images/ros2讲义（附前置知识）-image-10.png)

tightVNC需要在狗子和自己的电脑上分别安装，安装方法也有一定的差异性，在此分别进行解释。

**在自己电脑上安装**：https://www.tightvnc.com/download.php，像安装普通软件一样

### 在狗子里面安装

在狗子配置则会**相对复杂**，需按照以下步骤进行，提到的命令都应该在狗子的终端内执行。

**安装软件本体：**

```plain&#x20;text
sudo apt install tightvncserver #下载tightvncserver 

vncpasswd #设置tightvnc的密码命令：#建议设置的简单一点，比如12345678

(a). tightvncserver
(b). tightvncserver -kill :1
#启动并结束一个tightvncserver，从而生成配置文件：
```

**修改配置文件：**(对配置文件进行修改，此步骤较为复杂，请务必认真操作)

```markdown
(a). 用终端内的文本编辑器vim（或nano，视狗子情况而定）打开配置文件：
vim（nano） ~/.vnc/xstartup

(b). 将其中的内容修改为：
#!/bin/sh
# xrdb $HOME/.Xresources
xsetroot -solid grey 
# x-terminal-emulator -geometry 80x24+10+10 -ls -title "$VNCDESKTOP Desktop" &
# x-window-manager & 
# Fix to make GNOME work export XKL_XMODMAP_DISABLE=1 
autocutsel -fork 
openbox &
# /etc/X11/Xsession 
/usr/bin/lxsession Lubuntu -e LXDE & 
```

于是，我们就配置好了两边的tightVNC。接下来就可以进行连接了。

### **连接：**

连接操作的步骤如下：

**在狗子的终端里**开启tightVNC，使用指令：

**tightvncserver :1    #出现下图即为成功**

![](images/ros2讲义（附前置知识）-image-11.png)

然后

在电脑端tightVNC连接狗子的远程桌面。步骤如下：

(a). 打开已经安装的tightVNC软件。

(b). 在Remote Host里填入自己狗子的IP地址，并附加端口号5901（这是默认配置）。

(c). 点击connect，输入刚才设置好的密码。

看到下图即为成功

![](images/ros2讲义（附前置知识）-92158203fb81f3b76c9f0160122eae2b.png)



以上就是进行远程连接相关操作的教程。

如果仍然无法成功连接，请及时与我们联系，我们将配合解决相关问题💗



## 1.6科普：ros2

**ROS2**：机器人操作系统 2，为机器人开发提供标准化的框架（节点通信、硬件驱动、算法封装），支持多语言（C++/Python）、多平台（Linux/Windows/ 嵌入式）。

**核心优势**：去中心化通信（DDS）、更好的实时性、跨平台兼容。

**设计目的**：简化在各种机器人平台上创建复杂而强大的机器人行为的任务即不重复造轮子
**核心概念**：

* 节点（Node）：最小的执行单元（如一个节点控制电机、一个节点处理激光雷达）；

* 功能包（Package）：组织节点 / 配置 / 脚本的标准化文件夹；

* 工作空间（Workspace）：存放多个功能包的容器（包含 src / 编译 / 安装目录）。

***







# 2.给狗起一个好听的名字



打开下面两个文件夹

/opt/ros2/cyberdog/share/cyberdog\_bringup/bringup/manual.py

/opt/ros2/cyberdog/lib/python3.6/site-packages/mi/cyberdog\_bringup/manual.py&#x20;

将函数get\_namespace均进行如下的修改

（可以把custom\_namespace改成你喜欢的名字，比如dog1或者dog2）

![](images/ros2讲义（附前置知识）-image-13.png)

重启机器狗即可（在终端输入sudo reboot）







# 3.节点

节点是 ROS2 中**最小的可执行程序**，每个节点负责一个独立功能（如 “发布速度指令”“订阅激光雷达数据”“控制摄像头”）。

* 节点之间通过「话题 / 服务 / 参数」通信，互不干扰，便于调试和复用；

* 一个功能包可以包含多个节点，一个节点也可以单独成包。

#### 3.1准备**工作空间**

（机器人开发中存放所有开发资料（各种编写的代码、参数、脚本等文件）的文件夹，开发过程的大本营。

工作空间的名称可自定义，数量也可不唯一。）

```plain&#x20;text
mkdir -p ros2_ws/src #这里取了一个叫ros2_ws的用于讲解ros2，建议先保持一样，后面的代码就可以直接复制用
cd ros2_ws/src     #src是给我们写的代码空间
```

#### 3.2创建功能包

```plain&#x20;text
# 完整命令：创建ament_python类型的demo_python_pkg包，依赖rclpy
ros2 pkg create demo_python_pkg --build-type ament_python --dependencies rclpy example_interfaces --license Apache-2.0
```

然后你就看到了这些东西（右图）

![](images/ros2讲义（附前置知识）-image-14.png)



#### 3.3编写你的第一个节点

在/ros2\_ws/src/demo\_python\_pkg下新建node\_helloworld.py（可以鼠标操作，也可以复习一下命令行）

```plain&#x20;text
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
```

#### 3.4修改setup.py

修改setup.py，找到 `entry_points` 字段（通常在文件末尾），添加新节点的入口：

```plain&#x20;text
entry_points={
        'console_scripts': [
             "hello = demo_python_pkg.node_helloworld:main",  # 新增！你的新节点，记得加上逗号
        ],
    },
```

![](images/ros2讲义（附前置知识）-image-15.png)

#### 3.5编译功能包（colcon build）  回到工作区！

```plain&#x20;text
# 回到工作空间根目录（关键！不能在src/或包目录下编译）
cd ~/ros2_ws
# 编译指定包（效率更高，只编译你修改的包）
colcon build --packages-select demo_python_pkg
# 若要编译所有包，直接执行：colcon build
```

* &#x20;build，编译空间，保存编译过程中产生的**中间文件**；

* install，安装空间，放置编译得到的可执行文件和脚本；

* log，日志空间，编译和运行过程中，保存各种警告、错误、信息等**日志**。

* src，代码空间，要编写的代码、脚本，都需要人为的放置到这里；



值得注意的是，我们后面实际上运行的是被复制到install目录下的功能包（可以看到hello变成了一个可执行文件）



![](images/ros2讲义（附前置知识）-image-16.png)



#### 3.6加载环境变量（source，必做！）

编译后的节点需要加载环境变量才能被 ROS2 找到，**每次新开终端 / 编译后都要执行**：

```plain&#x20;text
# 在工作空间根目录执行
source install/setup.sh
```

#### 3.7运行新节点

执行 `ros2 run` 命令，格式：`ros2 run 功能包名 节点运行名`：

```plain&#x20;text
ros2 run demo_python_pkg hello
```

![](images/ros2讲义（附前置知识）-image-17.png)

即可看到打印了很多次hello，world(该图显示的是在我自己电脑上，不是在机器狗上)

#### 3.8和节点有关的命令

```plain&#x20;text
# 1. 列出所有正在运行的节点名
ros2 node list

# 2. 查看某个节点的详细信息 (发布/订阅了哪些话题)
ros2 node info /<node_name>

# 3. 查找某个节点的所有参数列表
ros2 param list /<node_name>

# 4. 获取某个具体参数的值 
ros2 param get /<node_name> <parameter_name>

# 5. 启动一个节点 (使其进入“运行中”状态)
ros2 run <package_name> <executable_name>
```

***



#### 3.9补充：xml文件是什么

`package.xml` 是一个采用 **XML 格式** 的元数据文件。它的核心作用是向 ROS 2 的编译工具（如 `colcon`）和包管理工具（如 `rosdep`）描述这个软件包的**基础信息**和**依赖关系**。

**它的三大核心职能**

* **身份定义**：规定了包的唯一名称、版本号、作者以及谁在维护它。

* **依赖声明（最重要）**：列出这个包运行或编译时必须安装的其他库。比如你的代码里用了超声波数据（`sensor_msgs`），就必须在这里“挂号”。

* **生态接入**：让你的包能被 ROS 2 整个生态系统搜索到，并允许自动化工具一键安装所有配套环境。

特别需要注意的地方：

**`<name>`**: 软件包名。必须与文件夹名称一致。

**`<depend>`**: 通用依赖。表示编译和运行都需要这个库。

后面我们会在避障的项目里面看到修改package.xml的时候



# 4.话题  发布-订阅机制

话题 = **节点之间发消息的通道**

话题数据传输的特性是从一个节点到另外一个节点，发送数据的对象称之为**发布者（publisher）**，接收数据的对象称之为**订阅者（subscriber）**，每一个话题都需要有一个名字，传输的数据也需要有固定的数据类型。

话题使用 .msg 文件定义消息的数据格式

### 1️⃣简单的：helloworld

可以用下面这两个文件体验一下，在一个终端ros2 run demo\_python\_pkg helloworld\_sub，一个终端ros2 run demo\_python\_pkg helloworld\_pub

！记得要先修改set.py和source激活

![](images/ros2讲义（附前置知识）-image-18.png)



[topic\_helloworld\_pub.py](files/ros2讲义（附前置知识）-topic_helloworld_pub.py)

[topic\_helloworld\_sub.py](files/ros2讲义（附前置知识）-topic_helloworld_sub.py)





常用命令：需要新建一个终端，然后source install/setu&#x70;**.bash**（或者.sh），然后输命令

```markdown
# 1. 概览：查看当前系统中所有正在活跃的话题
ros2 topic list

# 2. 查户口：查看某个特定话题的类型，以及有多少个发布者和订阅者
ros2 topic info /<topic_name>

# 3. 查字典：查看某一种消息格式里面到底包含哪些字段
ros2 interface show <msg_type>

# 4. 监听：在终端里实时滚动打印某个话题正在传输的数据（按 Ctrl+C 停止）
ros2 topic echo /<topic_name>

# 5. 测速：测量并显示某个话题的数据发布频率（每秒发多少次，即 Hz 数）
ros2 topic hz /<topic_name>

# 6. 发送：手动向某个话题发布消息（注意：数据必须是 YAML 字典格式！）
ros2 topic pub <topic_name> <msg_type> "{<字段名>: <具体内容>}"
# 例如：ros2 topic pub /topic_helloworld std_msgs/msg/String "{data: 'nihao'}"
#   - 常用附加参数： --rate 1 (代表以 1Hz 的频率持续发送)
#   - 常用附加参数： -1     (代表只发 1 次就退出)
```

补充一个命令：rqt

**rqt** 是一个基于 Qt 开发的**图形化界面（GUI）工具框架**

![]()

### 2️⃣有趣的：机器狗避障

本项目通过两个 Python 节点展示了 ROS 2 的**话题 (Topic) 通信机制**，实现了“感知、决策 、执行”的完整链路。

![](images/ros2讲义（附前置知识）-image-20.png)



[ultrasonic\_move.py](files/ros2讲义（附前置知识）-ultrasonic_move.py)

[ultrasonic\_sub.py](files/ros2讲义（附前置知识）-ultrasonic_sub.py)

注意：把里面的狗的名字改成自己的（原本叫“Tiedan”），然后修改好seetup.py和**package.xml**，然后source install/setup.py

**修改package.xml为下面的内容（必做！！！）**

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>demo_python_pkg</name>
  <version>0.0.0</version>
  <description>Ultrasonic obstacle avoidance for CyberDog</description>
  
  <maintainer email="mi@todo.todo">mi</maintainer>
  
  <license>Apache-2.0</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>protocol</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```







# 5.服务 (Service)

在 ROS 2 中，服务（Service）是一种**基于“客户端-服务端”（Client/Server）模型**的双向通信机制。

* **客户端（Client）：** 需要时主动发起请求（Request），然后等待服务端的处理结果（Response）。

* **服务端（Server）：** 提供特定功能，平时处于待命状态（不占用网络带宽，不主动发数据）。一旦收到请求，就开始处理，处理完必须给个回复。

![](images/ros2讲义（附前置知识）-image-21.png)



![](images/ros2讲义（附前置知识）-image-22.png)

和话题通信类似，服务通信的核心还是要传递数据，话题使用 .msg 文件定义，服务使用的是 .srv 文件定义。

这是两个很简单的文件，client发送两个数字，server接受后进行sum，传回给client

**！需要注意的是：ros2 run demo\_python\_pkg client\_add 3 4 （运行client的时候需要在同一行把两个数字传过去） 因为：代码里用的是 `sys.argv`，而不是 `input()`**

[service\_server\_add.py](files/ros2讲义（附前置知识）-service_server_add.py)



[service\_client\_add.py](files/ros2讲义（附前置知识）-service_client_add.py)

```python
# 1. 查黄页：列出当前系统中所有处于待命状态的服务<service_name>
ros2 service list
# ros2 service list 会显示/add_two_ints

# 2. 查格式：查看某个具体服务使用的是哪种服务接口类型<service_type>
ros2 service type <service_name>
#ros2 service type /add_two_ints 会显示 example_interfaces/srv/AddTwoInts

# 3. 看细节：查字典，看看这个接口类型具体包含哪些 Request 和 Response 字段
ros2 interface show <service_type>

# 4. 万能打电话：不写代码，直接在终端里作为客户端发起服务请求
# （⚠️ 注意：请求数据必须严格使用带有大括号和参数名的 YAML 字典格式！）
ros2 service call <service_name> <service_type> "{<field_name>: <value>}"
# 例如：ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 10, b: 20}"
```

&#x20;

# **6.遇到问题解决措施**

核心步骤：先自查→再问AI→后查文档→最后求助硬件部，减少重复答疑。

## 一、自查步骤

1. **环境与操作自查**

   * SSH/VNC连接：终端输`ping 机器狗IP`，不通则检查WiFi（Cyberdog，密码12345678）和IP。

   * VS Code：确认1.98版、未自动更新；打不开则修改Code文件夹为MyCode（路径见前文）。

   * 编译与环境：在ros2\_ws根目录编译（`pwd`确认路径），编译后必输`source install/setup.sh`；改代码后需重新编译对应包（`colcon build --packages-select 功能包名`）。

   * 节点/话题/服务：运行前确认编译和环境加载，用`ros2 node/topic/service list`查看状态。

2. **代码与配置自查**

   * setup.py/package.xml：确认格式正确、依赖完整、包名与文件夹一致。

   * Vim操作：修改后需`Esc→:wq`保存。

   * 机器狗命名：修改两个manual.py后，需`sudo reboot`重启。

## 二、AI求助

清晰描述问题，格式：场景+操作步骤+完整报错+已尝试操作，示例见下文。

示例：机器狗ROS2开发，编译demo\_python\_pkg时输`colcon build --packages-select demo_python_pkg`，报错“ImportError: No module named 'rclpy'”，已检查package.xml依赖，求解决。

## 三、查阅答疑文档

AI求助无果后，查阅往届培训PPT、智能狗课程PPT（已提供）。

查询本届答疑文档（会发在选手群里）

## 四、向硬件部求助

求助需提供以下完整信息（附终端输出截图/内容）：

1. **基础信息**：机器狗编号/用户名、IP、具体问题场景。

2. **终端输出**：网络（`ping 机器狗IP`、`ifconfig`）、编译/节点/VNC相关指令输出、完整报错。

3. **已尝试操作**：自查、AI求助、查文档的具体情况。

求助方式：选手群或讲师微信（a19870561520），避免模糊描述。

## 五、高频问题快速自查

* “command not found”：未编译或未加载环境（`source install/setup.sh`）。

* 依赖缺失：检查package.xml，或输`sudo apt update && sudo apt install 缺失依赖`。

* SSH连不上：检查WiFi、IP、密码（123）。

* VNC连不上：机器狗未启动服务（`tightvncserver :1`）或IP格式错误。
