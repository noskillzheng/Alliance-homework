## 导入

`motor_simulator`是一个ROS2包，直接拉下来放在对应的位置

## 使用

启动模拟电机节点

```
ros2 run motor_simulator motor_simulator_node    
```

启动后用`ros2 topic list`会显示电机的各个参数的topic名字

angle、velocity、torque是状态。control_torque是输入到电机的控制扭矩

为velocity的发布值添加了噪声，只是图省事没有弄成正态分布而是随机分布

三秒没有接收到控制数据会自动重置成初始状态

## 建议

搭配可视化工具，比如foxglove

可能有bug，遇到请反馈