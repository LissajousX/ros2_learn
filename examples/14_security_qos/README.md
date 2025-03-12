# 第14章：安全和QoS示例代码

本目录包含了ROS2安全特性和QoS（服务质量）设置的示例代码，用于配合第14章的学习材料。

## 文件说明

### QoS示例

1. `qos_publisher.py` - 演示不同QoS设置的发布者
2. `qos_subscriber.py` - 演示不同QoS设置的订阅者
3. `qos_compatibility_test.py` - 测试不同QoS配置之间的兼容性
4. `qos_monitor.py` - QoS监控工具，用于监视和分析ROS2系统中的QoS设置和性能

### 安全示例

1. `security_demo.py` - ROS2安全通信演示
2. `security_policy.xml` - 安全策略文件
3. `setup_security.sh` - 设置ROS2安全环境的脚本

## 使用方法

### 运行QoS示例

1. 启动QoS发布者：

```bash
python3 qos_publisher.py
```

2. 在另一个终端中启动QoS订阅者：

```bash
python3 qos_subscriber.py
```

3. 运行QoS兼容性测试：

```bash
python3 qos_compatibility_test.py
```

4. 运行QoS监控工具：

```bash
python3 qos_monitor.py
```

### 运行安全示例

1. 首先设置安全环境：

```bash
bash setup_security.sh
```

2. 按照脚本输出的指示，在两个不同的终端中运行安全通信的发布者和订阅者。

## 注意事项

- 运行安全示例前需要安装SROS2工具包：`sudo apt install ros-humble-sros2`
- QoS监控工具需要安装matplotlib：`pip install matplotlib`
- 确保所有Python脚本都有执行权限：`chmod +x *.py`
