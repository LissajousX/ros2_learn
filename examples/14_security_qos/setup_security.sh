#!/bin/bash

# 这个脚本用于设置ROS2安全环境并运行安全演示

# 确保脚本在出错时停止
set -e

# 创建安全密钥库目录
KEYSTORE_DIR=~/ros2_security_keystore
echo "创建安全密钥库: $KEYSTORE_DIR"
rm -rf $KEYSTORE_DIR
ros2 security create_keystore $KEYSTORE_DIR

# 创建节点密钥
echo "为talker节点创建密钥"
ros2 security create_key $KEYSTORE_DIR /security_demo/secure_talker

echo "为listener节点创建密钥"
ros2 security create_key $KEYSTORE_DIR /security_demo/secure_listener

# 应用安全策略
echo "应用安全策略"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ros2 security create_permission $KEYSTORE_DIR /security_demo/secure_talker $SCRIPT_DIR/security_policy.xml
ros2 security create_permission $KEYSTORE_DIR /security_demo/secure_listener $SCRIPT_DIR/security_policy.xml

echo "安全环境设置完成！"
echo ""
echo "要运行安全演示，请在两个单独的终端中执行以下命令："
echo ""
echo "终端1 (talker):"
echo "export ROS_SECURITY_KEYSTORE=$KEYSTORE_DIR"
echo "export ROS_SECURITY_ENABLE=true"
echo "export ROS_SECURITY_STRATEGY=Enforce"
echo "python3 $SCRIPT_DIR/security_demo.py --ros-args -e use_security:=true -e security_enclave:=/security_demo/secure_talker"
echo ""
echo "终端2 (listener):"
echo "export ROS_SECURITY_KEYSTORE=$KEYSTORE_DIR"
echo "export ROS_SECURITY_ENABLE=true"
echo "export ROS_SECURITY_STRATEGY=Enforce"
echo "python3 $SCRIPT_DIR/security_demo.py --listener --ros-args -e use_security:=true -e security_enclave:=/security_demo/secure_listener"
