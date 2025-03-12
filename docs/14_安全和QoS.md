# 第14章：安全和QoS

## 1. ROS2安全特性概述

### 1.1 为什么ROS2需要安全特性

ROS1在设计时主要考虑的是在受信任的网络环境中运行，缺乏内置的安全机制。而ROS2从设计之初就考虑了安全性，主要原因包括：

- 机器人系统越来越多地部署在开放环境中
- 工业和医疗等领域对安全性有严格要求
- 联网机器人面临的网络安全威胁日益增加
- 机器人系统可能处理敏感数据

### 1.2 ROS2安全架构

ROS2的安全特性主要基于以下几个方面：

- **DDS安全扩展**：利用DDS-Security规范提供的安全机制
- **SROS2**：ROS2安全工具包，提供密钥管理和配置工具
- **访问控制**：基于节点和话题的细粒度访问控制
- **加密通信**：节点间通信的加密保护
- **认证机制**：确保只有授权的节点才能参与通信

## 2. DDS安全基础

### 2.1 DDS-Security规范

DDS-Security是对DDS标准的安全扩展，提供了五个主要的安全插件：

1. **认证插件**（Authentication）：验证通信实体的身份
2. **访问控制插件**（Access Control）：控制对DDS资源的访问权限
3. **加密插件**（Cryptographic）：提供消息加密和解密功能
4. **日志插件**（Logging）：记录安全相关事件
5. **数据标记插件**（Data Tagging）：为数据添加安全标记

### 2.2 DDS安全模型

DDS安全模型基于PKI（公钥基础设施）：

- 每个参与者（participant）都有自己的公钥/私钥对
- 通过CA（证书颁发机构）签发的证书建立信任关系
- 使用共享密钥进行高效的数据加密

## 3. SROS2工具包

### 3.1 SROS2概述

SROS2（Secure ROS2）是ROS2的安全工具包，提供：

- 密钥和证书管理工具
- 安全配置生成工具
- 与ROS2命令行工具的集成

### 3.2 安装SROS2

```bash
# 安装SROS2工具包
sudo apt update
sudo apt install ros-humble-sros2
```

### 3.3 基本使用流程

1. **创建安全密钥库**：

```bash
ros2 security create_keystore ~/ros2_security_keystore
```

2. **为节点创建密钥和证书**：

```bash
ros2 security create_key ~/ros2_security_keystore /talker_listener/talker
ros2 security create_key ~/ros2_security_keystore /talker_listener/listener
```

3. **创建访问控制策略**：

```bash
ros2 security create_permission ~/ros2_security_keystore /talker_listener/talker policies/sample_policy.xml
ros2 security create_permission ~/ros2_security_keystore /talker_listener/listener policies/sample_policy.xml
```

4. **启用安全运行节点**：

```bash
export ROS_SECURITY_KEYSTORE=~/ros2_security_keystore
export ROS_SECURITY_ENABLE=true
export ROS_SECURITY_STRATEGY=Enforce
ros2 run demo_nodes_cpp talker --ros-args -e use_security:=true -e security_enclave:=/talker_listener/talker
```

## 4. 访问控制策略

### 4.1 策略文件格式

访问控制策略使用XML格式定义，示例：

```xml
<?xml version="1.0" encoding="UTF-8"?>
<policy version="0.2.0">
  <enclaves>
    <enclave path="/talker_listener/talker">
      <profiles>
        <profile ns="/" node="talker">
          <topics publish="ALLOW" subscribe="ALLOW">
            <topic>chatter</topic>
          </topics>
        </profile>
      </profiles>
    </enclave>
    <enclave path="/talker_listener/listener">
      <profiles>
        <profile ns="/" node="listener">
          <topics publish="DENY" subscribe="ALLOW">
            <topic>chatter</topic>
          </topics>
        </profile>
      </profiles>
    </enclave>
  </enclaves>
</policy>
```

### 4.2 权限控制级别

- **话题级别**：控制节点对特定话题的发布/订阅权限
- **服务级别**：控制节点对特定服务的调用/提供权限
- **参数级别**：控制节点对参数的读/写权限
- **命名空间级别**：控制节点对特定命名空间的访问权限

## 5. QoS（服务质量）

### 5.1 QoS概述

QoS（Quality of Service）是ROS2中的一个核心概念，允许用户配置数据传输的各种特性：

- 可靠性：数据是否必须全部送达
- 持久性：数据是否需要保存给后来的订阅者
- 截止时间：数据的有效期
- 历史记录：保留多少历史消息
- 优先级：消息的传输优先级

### 5.2 QoS策略

#### 5.2.1 可靠性（Reliability）

- **RELIABLE**：确保所有数据都被传递，如果需要会重传
- **BEST_EFFORT**：尽力传递，但不保证，适合实时数据

#### 5.2.2 历史记录（History）

- **KEEP_LAST**：只保留最近的N条消息（N由depth参数指定）
- **KEEP_ALL**：保留所有消息，直到资源耗尽

#### 5.2.3 持久性（Durability）

- **VOLATILE**：不为晚加入的订阅者保存数据
- **TRANSIENT_LOCAL**：为晚加入的订阅者保存数据

#### 5.2.4 截止时间（Deadline）

指定发布者应该以什么频率发布数据，或订阅者期望以什么频率接收数据

#### 5.2.5 活跃度（Liveliness）

- **AUTOMATIC**：只要参与者还活着，实体就被认为是活跃的
- **MANUAL_BY_PARTICIPANT**：参与者必须定期断言其活跃性
- **MANUAL_BY_TOPIC**：每个实体必须定期断言其活跃性

### 5.3 QoS配置文件

ROS2提供了几个预定义的QoS配置文件：

- **Default**：系统默认设置
- **Sensor Data**：适合传感器数据的设置（BEST_EFFORT可靠性）
- **Services**：适合服务调用的设置（RELIABLE可靠性）
- **Parameters**：适合参数服务的设置
- **System Default**：使用底层DDS的默认设置

## 6. 在代码中使用QoS

### 6.1 C++中配置QoS

```cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("qos_example");
  
  // 创建自定义QoS配置
  rclcpp::QoS qos_profile(10);
  qos_profile.reliability(rclcpp::ReliabilityPolicy::RELIABLE);
  qos_profile.durability(rclcpp::DurabilityPolicy::TRANSIENT_LOCAL);
  qos_profile.deadline(std::chrono::milliseconds(200));
  
  // 使用QoS配置创建发布者
  auto publisher = node->create_publisher<std_msgs::msg::String>(
    "topic", qos_profile);
  
  // 使用预定义的QoS配置文件
  auto sensor_pub = node->create_publisher<std_msgs::msg::String>(
    "sensor_topic", rclcpp::SensorDataQoS());
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
```

### 6.2 Python中配置QoS

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.qos import QoSReliabilityPolicy, QoSDurabilityPolicy
from std_msgs.msg import String

def main():
    rclpy.init()
    node = Node("qos_example")
    
    # 创建自定义QoS配置
    qos_profile = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=10
    )
    
    # 使用QoS配置创建发布者
    publisher = node.create_publisher(
        String, 'topic', qos_profile)
    
    # 使用预定义的QoS配置文件
    from rclpy.qos import qos_profile_sensor_data
    sensor_pub = node.create_publisher(
        String, 'sensor_topic', qos_profile_sensor_data)
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 6.3 QoS兼容性

发布者和订阅者的QoS设置必须兼容才能成功通信：

- 如果发布者使用BEST_EFFORT，订阅者也必须使用BEST_EFFORT
- 如果发布者使用VOLATILE，订阅者不能使用TRANSIENT_LOCAL
- 发布者的截止时间必须小于或等于订阅者的截止时间

## 7. 网络配置和优化

### 7.1 DDS发现机制

DDS使用简单参与者发现协议（SPDP）和端点发现协议（SEDP）：

- 参与者通过多播UDP包宣布自己的存在
- 发现后通过单播UDP交换详细信息
- 建立P2P连接进行数据传输

### 7.2 网络配置

#### 7.2.1 配置DDS域ID

```bash
export ROS_DOMAIN_ID=<0-232>
```

不同域ID的ROS2节点不会互相发现和通信

#### 7.2.2 配置多播

```bash
# 禁用多播（在某些网络环境中可能需要）
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=no_multicast.xml
```

`no_multicast.xml`示例：

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <participant profile_name="participant_profile" is_default_profile="true">
        <rtps>
            <builtin>
                <metatrafficUnicastLocatorList>
                    <locator/>
                </metatrafficUnicastLocatorList>
                <initialPeersList>
                    <locator>
                        <udpv4>
                            <address>192.168.1.100</address>
                        </udpv4>
                    </locator>
                </initialPeersList>
            </builtin>
        </rtps>
    </participant>
</profiles>
```

### 7.3 性能优化

#### 7.3.1 选择合适的DDS实现

ROS2支持多种DDS实现，各有优缺点：

- **Fast DDS**（默认）：功能全面，性能良好
- **Cyclone DDS**：高性能，低资源消耗
- **Connext DDS**：商业级实现，功能丰富

```bash
# 切换DDS实现
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

#### 7.3.2 共享内存传输

同一主机上的节点可以使用共享内存通信，大幅提高性能：

```bash
# 启用Fast DDS共享内存传输
export FASTRTPS_DEFAULT_PROFILES_FILE=shm_profile.xml
```

`shm_profile.xml`示例：

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <transport_descriptors>
        <transport_descriptor>
            <transport_id>shm_transport</transport_id>
            <type>SHM</type>
        </transport_descriptor>
    </transport_descriptors>

    <participant profile_name="participant_profile" is_default_profile="true">
        <rtps>
            <userTransports>
                <transport_id>shm_transport</transport_id>
            </userTransports>
        </rtps>
    </participant>
</profiles>
```

## 8. 安全最佳实践

### 8.1 安全规划

1. **进行威胁建模**：识别潜在威胁和风险
2. **确定安全需求**：基于威胁模型确定所需的安全级别
3. **分层安全策略**：实施多层次的安全防护

### 8.2 实施建议

1. **最小权限原则**：节点只获取完成任务所需的最小权限
2. **网络隔离**：使用防火墙和VLAN隔离机器人网络
3. **定期更新**：保持ROS2和依赖库的更新
4. **监控和日志**：实施安全监控和日志记录
5. **安全启动**：确保系统以安全的方式启动

### 8.3 常见安全陷阱

1. **硬编码凭证**：避免在代码中硬编码密钥和凭证
2. **忽略证书验证**：始终验证证书的有效性
3. **过度开放的权限**：避免给节点过多不必要的权限
4. **忽略物理安全**：确保机器人的物理访问安全
