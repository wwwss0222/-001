# 第二十六届中国机器人及人工智能大赛智慧药房组
# 沿用前人经验 特此感谢GitHub上的一位老哥
## 需要启动的功能包 (Master)
 * `roslaunch robot_navigation robot_navigation.launch`
 * `roslaunch actuator car_master.launch ` #最终版主机代码
 * `roslaunch char_recognizer char_recognizer.launch`         # 真图 识别图像
 * `roslaunch deliver_scheduler scheduler.launch` #定时器
 * Master 可独立工作，完成**配送、识别标准数字**，并且可以自动修改周期为小哥周期 2，药品周期 
## 需要启动的功能包 (Slave)
 * `roslaunch robot_navigation robot_navigation.launch` #最终版从机代码
 * `roslaunch actuator car_slave.launch `
 * Slave 需依赖 Master，在 EveryoneStatus 中监听到相关状态，方开始工作。仅负责**配送**。

# 因第二十六届中国机器人及人工智能大赛智慧药房规则修改 故仅用双车
# new_drug_master（存放主机代码文件）下主要为actuator（负责主机运行逻辑）、char_recognizer（负责识别任务）、deliver_scheduler(负责定时)
# new_drug_slave（存放从机代码文件）下主要为actuator（负责从机运行逻辑）
# 运行逻辑代码为actuator\script下final_master.py（主机）以及final_slave.py(从机)
# 不要提为什么代码里那么多奇奇怪怪的reset或者保护机制 悄悄告诉你 小车性能跟不太上哦
