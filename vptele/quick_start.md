1. visionpro 翻到第二页，tracking streamer 点击 start
2. visionpro 通过wifi连着我座位上的路由器， rm机器人通过wifi连着学校的路由器，丢包和延迟
3. ssh 连接 rm


操控并且记录轨迹
``` bash
cd /home/rm/thomasyht/vptele
bash dual_arm_control_old.sh
```

快速回放上一次的轨迹
``` bash
bash dual_arm_playback.sh
```

回放指定轨迹
``` bash
bash play_dual_example.sh
```


# 单臂+灵巧手教程

1. 首先要有一台电脑通过网络（wifi或者网线）连接单机械臂，wifi的话是连接单臂的热点，网线的话是电脑直连单臂的网口，然后将网段修改到和单臂一样，具体是什么我忘了，可以上官网看一下。
    https://develop.realman-robotics.com/robot/quickUseManual/ 建议是过一遍这个网页，对操控睿尔曼单臂很重要。尤其里面的开关机，连接控制，示教器等有助于快速调试。

2. 在机械臂开机，能正常连接操控之后，在自己的电脑上配好环境后，运行这个项目，命令如下：
```bash
python main.py --config config/config.yaml
```

操控灵巧手的话就在 config/config.yaml 的 end_effector: 里面填上hand就行了，如果填none，就会单独控制机械臂。

操控单臂有几个核心区别可能需要注意：
1. robot_ip和双臂相比变了，因为整个操控系统的网络拓扑不同了，本来是开发板连着两个机械臂，两个臂的ip是169.254.128.18 169.254.128.19，但现在是192.168.1.18
2. arm_model变了，这个影响ik逆解用到的模型，发送关节角用到的api等
3. 初始位置和安全范围会有所区别，因为单臂是夹在桌子上的，所以操控范围不太一样。
4. 操控单臂需要更小心，因为桌子很晃，可能会掀翻，稍加谨慎。