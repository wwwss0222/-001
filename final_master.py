#! /usr/bin/env python
# -*- coding: utf-8 -*-

import threading
from actuator_points import point_ABC_master, point_1234_master, point_special_master
import actionlib
import rospy
from tf_conversions import transformations
from math import pi
import tf
from geometry_msgs.msg import Point, Quaternion, Pose, PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from playsound import playsound
from statemachine import State, StateMachine
from actuator.srv import (
    DestinationMsg,
    DestinationMsgRequest,
    PermissionMsg,
    PermissionMsgResponse,
    NumRecognizeMsg,
    NumRecognizeMsgRequest,
    GomissionMsg,
    GomissionMsgRequest,
    GomissionMsgResponse,
)
from actuator.msg import EveryoneStatus

status = "wait"

# 为后续同时调用移动和控制和图像识别做准备
def thread_CV():
    rospy.spin()

emptyResponse = GomissionMsgResponse()
emptyResponse = 1


class SimpleStateMachine(StateMachine):
    def __init__(self, assignment):
        self.assignment = assignment
        self.achieve_start_flag = False
        super(SimpleStateMachine, self).__init__()
        rospy.set_param('status_start', 'start') #设置目前节点状态
    Init = State("Init", initial=True)
    Re_char = State("Re_char")
    GO_ABC = State("GO_ABC")
    GO_1234 = State("GO_1234")
    Before_Starting_Point = State("Before_Starting_Point")
    Starting_Point = State("Starting_Point")
    Fulfile = State("Fulfile")
    Starting_Point_new = State("Starting_Point_new")
    Type_Recognize = State("Type_Recognize")
    Type_Recognize_new = State("Type_Recognize_new")
    Go = (
        Init.to(Init, cond="ReAskMission")
        | Init.to(Re_char, cond="Char_false")
        | Re_char.to(Init)
        | Init.to(GO_ABC, cond="GetingTarget")
        | GO_ABC.to(Type_Recognize)
        | Type_Recognize.to(GO_1234,cond="Recognizeover")
        | Type_Recognize.to(Type_Recognize_new)
        | Type_Recognize_new.to(GO_1234)
        | GO_1234.to(Before_Starting_Point)
        | Before_Starting_Point.to(Starting_Point)
        | Starting_Point.to(Fulfile,cond="achieve_start") # ,cond="show_time"
        | Starting_Point.to(Starting_Point_new)
        | Starting_Point_new.to(Fulfile)
        | Fulfile.to(Init)
    )
    # 请求成功，等候识别
    def GetingTarget(self):
        if not hasattr(self, 'first_call'):
            # 第一次调用，将 Mgo_service 设置为 True
            self.assignment.mas_go_flag = True
            setattr(self, 'first_call', False)  # 设置标志表示不是第一次调用
        if self.assignment.asksuccess_flag and self.assignment.mas_go_flag:
            while not self.assignment.seefinished_flag:
                rospy.loginfo("虽然请求成功，但要识别，正在等候")
                rospy.sleep(3)
            # self.assignment.mas_go_flag = False
            return True
        else:
            return False
        
    def ReAskMission(self):
        if not self.GetingTarget():
            rospy.sleep(1)
            return True
        else:
            return False
        
    def Char_false(self):
        if self.assignment.char_blank:
            self.assignment.char_blank = False
            return True
        else:
            return False

    def Recognizeover(self):
        if self.assignment.Typerecognize_over:
            rospy.loginfo("识别完成")
            self.assignment.Typerecognize_over = False
            return True
        else:
            return False

    def achieve_start(self):
        if self.achieve_start_flag:
            return True
        else:
            return False
        
    def before_transition(self, state):
        Master_status = EveryoneStatus()
        Master_status.name = "Master"
        Master_status.status = state.id
        self.assignment.location_pub.publish(Master_status)
        # rospy.logwarn("请求从[%s]转移",state.id)

    def on_enter_Init(self):
        self.assignment.allow2see_flag = True
        # rospy.sleep(1)
        while not self.assignment.seefinished_flag:  # 标志位为False时，代表收到了新请求。
            if not self.assignment.logwarn_protect:  # False为不保护，第一次log
                self.assignment.logwarn_protect = True
                rospy.logwarn("接收到识别器请求，请原地等待")
        # 睡眠0.5s等待需求更新
        rospy.sleep(0.5)
        if self.assignment.mas_go_flag:
            self.assignment.master_ask_newtarget()
        self.assignment.logwarn_protect = False        

    def on_enter_Re_char(self):
        self.assignment.reset()
        rospy.set_param('status_start', 'start')
        self.assignment.allow2see_flag = False
        rospy.loginfo("前往重识别点")
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "world"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = point_ABC_master[
            self.assignment.mission_response.drug_location
        ]
        if self.assignment.master_go(goal) == True:
            print("到达重识别点")
            self.assignment.master_updateM()
            rospy.set_param('status_start', 'locationGo')
        else:
            rospy.set_param('status_start', 'locationGo')
            self.assignment.move_base_client.cancel_goal()

    def on_enter_GO_ABC(self):
        self.assignment.reset()
        rospy.set_param('status_start', 'start')
        self.assignment.allow2see_flag = False
        rospy.loginfo("前往配药区")
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "world"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = point_ABC_master[
            self.assignment.mission_response.drug_location
        ]
        if self.assignment.master_go(goal) == True:
            thread = threading.Thread(target=self.execute_after_arrivalABC)
            thread.start()
        else:
            thread = threading.Thread(target=self.execute_after_arrivalABC)
            thread.start()
            rospy.logerr("配药失败")
            self.assignment.move_base_client.cancel_goal()

    # 前往药物类型答题区
    def on_enter_Type_Recognize(self):
        rospy.loginfo("master前往药物类型识别区")
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "world"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = point_special_master[1]
        if self.assignment.master_go(goal) == True:
            rospy.loginfo("master到达药物类型识别区")
            self.assignment.type_ask_recognize()
        else:
            rospy.logerr("master前往药物类型识别区失败")
            self.assignment.move_base_client.cancel_goal()  # 取消当前目标导航点

    def on_enter_Type_Recognize_new(self):
        rospy.loginfo("master前往第二药物类型识别区")
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "world"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = point_special_master[2]
        if self.assignment.master_go(goal) == True:
            rospy.loginfo("master到达第二药物类型识别区")
            self.assignment.type_ask_recognize()
        else:
            rospy.logerr("master前往第二药物类型识别区失败")
            self.assignment.move_base_client.cancel_goal()  # 取消当前目标导航点

    def on_enter_GO_1234(self):
        rospy.loginfo("前往取药区")
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "world"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = point_1234_master[
            self.assignment.mission_response.deliver_destination
        ]
        if self.assignment.master_go(goal) == True:
            thread = threading.Thread(target=self.execute_after_arrival1234)
            thread.start()
        else:
            thread = threading.Thread(target=self.execute_after_arrival1234)
            thread.start()
            rospy.logerr("取药失败")
            self.assignment.move_base_client.cancel_goal()  # 取消当前目标导航点

    def on_enter_Before_Starting_Point(self):
        rospy.loginfo("master前往起点过渡点")
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "world"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = point_special_master[4]
        if self.assignment.master_go(goal) == True:
            rospy.loginfo("master到达起点过渡点")
        else:
            rospy.logerr("master前往起点过渡点")
            self.assignment.move_base_client.cancel_goal()  # 取消当前目标导航点

    def on_enter_Starting_Point(self):
        rospy.loginfo("前往起点")
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "world"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = point_special_master[0]
        if self.assignment.master_go(goal) == True:
            rospy.loginfo("到达起点")
            self.achieve_start_flag = True
            self.assignment.allow2see_flag = True
            rospy.set_param('status_start', 'locationGo')
            self.assignment.run()

        else:
            rospy.logerr("前往起点失败")
            rospy.set_param('status_start', 'locationGo')
            self.assignment.move_base_client.cancel_goal()  # 取消当前目标导航点

    def on_enter_Starting_Point_new(self):
        rospy.loginfo("前往第二起点")
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "world"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = point_special_master[3]
        if self.assignment.master_go(goal) == True:
            rospy.loginfo("到达第二起点")
            self.assignment.allow2see_flag = True
            rospy.set_param('status_start', 'locationGo')
            self.assignment.run()
        else:
            rospy.logerr("前往第二起点失败")
            rospy.set_param('status_start', 'locationGo')
            self.assignment.move_base_client.cancel_goal()  # 取消当前目标导航点

    def on_enter_Fulfile(self):
        self.assignment.MasterFulfile()
        rospy.loginfo("正在等候slave执行任务")
        self.achieve_start_flag = False
        self.assignment.mas_go_flag = False
        self.assignment.allow2see_flag = True

    def execute_after_arrivalABC(self):
    # 这里是之前代码中需要在新线程中执行的部分
        newABC = self.assignment.responseToABC[
            self.assignment.mission_response.drug_location
        ]
        rospy.logwarn("到达配药区%c", newABC)

        func_dict = {
            0: self.assignment.master_updateA,
            1: self.assignment.master_updateB,
            2: self.assignment.master_updateC,
        }
        key = self.assignment.mission_response.drug_location
        func_dict[key](self.assignment.mission_response.drug_location)

    def execute_after_arrival1234(self):
        rospy.logwarn("到达取药区%d", self.assignment.mission_response.deliver_destination)
        # setpose_instance.M_reset1234_pose(self.assignment.mission_response.deliver_destination)
        func_dict = {
            1: self.assignment.master_update1,
            2: self.assignment.master_update2,
            3: self.assignment.master_update3,
            4: self.assignment.master_update4,
        }
        # 获取键执行相应函数
        key = self.assignment.mission_response.deliver_destination
        if key in func_dict:
            func_dict[key]()

class Car_go(object):
    def __init__(self) :
        rospy.init_node('car_master')
        self.tf_listener = tf.TransformListener()
        self.pose = None
        self.got_pose = False  # 初始化标志为False
        self.pose_new = None
        self.got_pose_new = False
        self.asksuccess_flag = False  # 请求成功标志位
        self.seefinished_flag = False  # 识别结束标志位
        self.allow2see_flag = False  # 允许识别标志位,物理上的
        self.logwarn_protect = False
        self.Typerecognize_over = False
        self.fulfile_task = False
        self.mas_go_flag =False
        self.char_blank = False
        self.slave_location = 'Init'
        self.location_pub = rospy.Publisher("/location", EveryoneStatus, queue_size=10)
        self.location_sub = rospy.Subscriber("/location",EveryoneStatus,self.actuator_deallocation,queue_size=10)
        
        rospy.on_shutdown(self.master_shutdown)
        self.goal_states = ['PENDING', 'ACTIVE', 'PREEMPTED',   
                       'SUCCEEDED', 'ABORTED', 'REJECTED', 
                       'PREEMPTING', 'RECALLING', 'RECALLED', 
                       'LOST']

        self.permission_server = rospy.Service(
            "permission", PermissionMsg, self.master_dealCV_ask
        )
        rospy.loginfo("命令服务器正常启动")
        self.move_base_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("等待连接EPRobot1/move_base服务")

        # 等候60秒 不成功则退出
        self.move_base_client.wait_for_server(rospy.Duration(60))
        rospy.loginfo("成功连接EPRobot1/move base服务")

        self.mission_client = rospy.ServiceProxy("/EPRobot1/mission", DestinationMsg)
        rospy.loginfo("Car_master调度器客户端正常启动了")

        self.number_client = rospy.ServiceProxy(
            "/EPRobot1/mnumber_reminder_server", NumRecognizeMsg
        )

        self.Mgo_server = rospy.Service(
            "Mgo_server", GomissionMsg, self.Master_can_go
        )
        rospy.loginfo("msaster出发服务器正常启动")

        self.Sgo_client = rospy.ServiceProxy("/EPRobot2/Sgo_server", GomissionMsg)
        self.Sgo_client.wait_for_service()
        rospy.loginfo("Car_slave出发客户端启动")

        self.number_client.wait_for_service()
        rospy.loginfo("Car_master连上药品类型版识别请求服务器了")

        self.mission_client.wait_for_service()
        rospy.loginfo("Car_master连上调度器服务器了")

        self.mission_request = DestinationMsgRequest()
        self.num_request = NumRecognizeMsgRequest()
        self.go_request_2 = GomissionMsgRequest()

        self.responseToABC = {-1: "E", 0: "A", 1: "B", 2: "C", 3:"M"}
        add_thread = threading.Thread(target=thread_CV)
        add_thread.start()
        rospy.loginfo("deal CV thread OK")


    def master_shutdown(self):
        rospy.logerr("Stop the EPRobot1")
        self.move_base_client.cancel_goal()  # 取消当前目标导航点


    # 向服务器请求新任务,到达起点（标准数字区）
    def master_ask_newtarget(self):
        self.mission_request.request_type = 1  # 请求包编号为“请求新任务”
        self.mission_response = self.mission_client.call(
            0, self.mission_request.request_type, 0, 0
        )
        rospy.loginfo("代号:%d,请求新任务", 0)
        rospy.loginfo(
            "Get:%c,Send:%d",
            self.responseToABC[self.mission_response.drug_location],
            self.mission_response.deliver_destination,
        )
        if (
            self.mission_response.drug_location != -1
            and self.mission_response.deliver_destination != -1
        ):  # 不是负-1代表请求成功
            if self.mission_response.drug_location == 3: #没有识别到字母
                self.char_blank = True
            else:
                self.asksuccess_flag = True  # 请求成功
        else:  # 请求失败
            self.asksuccess_flag = False

    def Master_can_go(self,req):
        flag = req.request_go
        if flag == 1:
            self.mas_go_flag = True
        else:
            print("Master可以出发")
            self.mas_go_flag = False
        return self.mas_go_flag

    def MasterFulfile(self):
        self.go_response = self.Sgo_client.call(1)
        if self.go_response == 1:
            rospy.loginfo("命令slavec出发成功")
        else:
            rospy.loginfo("命令slave执行任务失败")

    # 向服务器上报已取药
    def master_updateA(self, newABC):
        playsound("/home/EPRobot/robot_ws/src/new_drug_master/music/dispenseA.mp3")
        self.mission_request.request_drug_type = newABC
        self.mission_request.request_type = 2  # 请求包编号为“完成配药/ABC”
        self.mission_client.call(0, self.mission_request.request_type, self.mission_request.request_drug_type, 0)  

    def master_updateB(self, newABC):
        playsound("/home/EPRobot/robot_ws/src/new_drug_master/music/dispenseB.mp3")
        self.mission_request.request_drug_type = newABC
        self.mission_request.request_type = 2  # 请求包编号为“完成配药/ABC”
        self.mission_client.call(0, self.mission_request.request_type, self.mission_request.request_drug_type, 0) 

    def master_updateC(self, newABC):
        playsound("/home/EPRobot/robot_ws/src/new_drug_master/music/dispenseC.mp3")
        self.mission_request.request_drug_type = newABC
        self.mission_request.request_type = 2  # 请求包编号为“完成配药/ABC”
        self.mission_client.call(0, self.mission_request.request_type, self.mission_request.request_drug_type, 0)   

    def master_updateM(self):
        self.mission_client.call(0, 2, 3, 0)
        self.mission_client.call(0, 3, 0, 0)

    # 向服务器上报已送药
    def master_update1(self):
        playsound("/home/EPRobot/robot_ws/src/new_drug_master/music/pick_up1.mp3")
        self.mission_request.request_type = 3  # 请求包编号为“完成送药/1234”
        self.mission_client.call(0, self.mission_request.request_type, 0, 0)

    def master_update2(self):
        playsound("/home/EPRobot/robot_ws/src/new_drug_master/music/pick_up2.mp3")
        self.mission_request.request_type = 3  # 请求包编号为“完成送药/1234”
        self.mission_client.call(0, self.mission_request.request_type, 0, 0)

    def master_update3(self):
        playsound("/home/EPRobot/robot_ws/src/new_drug_master/music/pick_up3.mp3")
        self.mission_request.request_type = 3  # 请求包编号为“完成送药/1234”
        self.mission_client.call(0, self.mission_request.request_type, 0, 0)

    def master_update4(self):
        playsound("/home/EPRobot/robot_ws/src/new_drug_master/music/pick_up4.mp3")
        self.mission_request.request_type = 3  # 请求包编号为“完成送药/1234”
        self.mission_client.call(0, self.mission_request.request_type, 0, 0)

    def master_finish(self):
        self.mission_request.request_type = 3  # 请求包编号为“完成送药/1234”
        self.mission_client.call(0, self.mission_request.request_type, 0, 0)

    def type_audio(self, number):
        audio_files = {
            0: "/home/EPRobot/robot_ws/src/new_drug_master/music/circle.mp3", # 圆形药丸
            1: "/home/EPRobot/robot_ws/src/new_drug_master/music/Elliptical.mp3", # 椭圆药丸
            2: "/home/EPRobot/robot_ws/src/new_drug_master/music/One-color.mp3", # 单色胶囊
            3: "/home/EPRobot/robot_ws/src/new_drug_master/music/Two-color.mp3", # 双色胶囊
            4: "/home/EPRobot/robot_ws/src/new_drug_master/music/fish.mp3" # 鱼肝油
        }
        audio_file = audio_files.get(number)
        if audio_file:
            playsound(audio_file)
        else:
            print("没有与该数字对应的音频文件")

    def type_ask_recognize(self):
        # 在药物类型版识别文件中 如果传回num_response为None，则numrecognize_over为false（识别失败），否则numrecognize_over为true（识别成功）
        drugs_type = {
            0:"圆形药丸",
            1:"椭圆药丸",
            2:"单色胶囊",
            3:"双色胶囊",
            4:"鱼肝油"
        }
        self.num_response = self.number_client.call(True)  
        rospy.loginfo("master请求识别药物类型版任务")
        if self.num_response.number != -1:  # 不是空代表识别成功 对不对我不知道
            self.Typerecognize_over = True
            print(self.num_response.number)
            drugs_type1 = drugs_type.get(self.num_response.number)
            rospy.loginfo("识别到药物类型：%s",drugs_type1)
            thread = threading.Thread(target=self.type_audio, args=(self.num_response.number,))
            thread.start()
        else:  # 识别失败
            self.Typerecognize_over = True
            rospy.logwarn("master识别药物类型版失败，进入下一任务")



    # 处理来自识别器的请求
    def master_dealCV_ask(self, req):
        if req.request == 0:  # "想看请求" request是从recognizer.py文件中传输的参数
            rospy.loginfo("接受:[想看]")
            self.seefinished_flag = False  # 未看完=想看=接受到想看请求=有新一轮
            if self.allow2see_flag:  # 已经到达识别区,允许识别
                self.allow2see_flag = False
                resp = PermissionMsgResponse(1)  # 可以看
                rospy.loginfo("回复:[可以]")
                rospy.set_param('status_start', 'start')
            else:
                resp = PermissionMsgResponse(0)  # 不可以
                rospy.loginfo("回复:[拒绝]")
                rospy.sleep(2)
        else:  # "看完了"
            rospy.loginfo("接受:[看完]")
            resp = PermissionMsgResponse(0)
            self.seefinished_flag = True  # 识别结束=不需要识别

        return resp

    def get_pose(self):
        try:
            # 等待tf变换可用
            self.tf_listener.waitForTransform('EPRobot1/map', 'EPRobot1/base_link', rospy.Time(), rospy.Duration(1.0))
            # 获取变换
            (trans, rot) = self.tf_listener.lookupTransform('EPRobot1/map', 'EPRobot1/base_link', rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.logerr("tf Error")
            return None
        # 将四元数转换为欧拉角
        euler = euler_from_quaternion(rot)
        # 假设我们只关心yaw（偏航角），这里我们使用1.57弧度作为示例
        yaw = euler[2]
        # 将欧拉角转换回四元数
        quaternion = quaternion_from_euler(0, 0, yaw)
        # 创建Point对象
        position = Point(*trans)
        # 创建Quaternion对象
        orientation = Quaternion(*quaternion)
        # 创建Pose对象
        pose = Pose(position, orientation)
        return pose

    def run(self):
        number = 0
        while not rospy.is_shutdown():
            self.pose = self.get_pose()
            if self.pose is not None:  # 如果获取到了有效的Pose对象
                print("成功获取")
                self.got_pose = True  # 设置标志为True
                break  # 退出循环
            elif number < 3:
                number = number + 1
            else:
                break
            rospy.sleep(0.5)  # Sleep for 1 second
            
    def run_new(self):
        number = 0
        while not rospy.is_shutdown():
            self.pose_new = self.get_pose()
            if self.pose_new is not None:  # 如果获取到了有效的Pose对象
                print("成功获取新位置")
                self.got_pose_new = True  # 设置标志为True
                break  # 退出循环
            elif number < 3:
                number = number + 1
            else:
                break
            rospy.sleep(0.5)  # Sleep for 1 second
            
    def reset(self):
        if self.got_pose:
            self.run_new()
            if self.got_pose_new:
                if abs(self.pose_new.position.x - self.pose.position.x) > 0.5 or abs(self.pose_new.position.y - self.pose.position.y) > 0.5:
                    flag = 0
                    initial_pose_pub = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=10)
                    loop_rate = rospy.Rate(10)
                    while flag < 5 :
                        pose_msg = PoseWithCovarianceStamped()
                        pose_msg.header.stamp = rospy.Time.now()
                        pose_msg.header.frame_id = "EPRobot1/map"
                        pose_msg.pose.pose = Pose(Point(self.pose.position.x, self.pose.position.y, 0), Quaternion(0.000, 0.000, self.pose.orientation.z, self.pose.orientation.w))
                        initial_pose_pub.publish(pose_msg)
                        # rospy.loginfo("Setting pose to: %s", pose_msg.pose.pose)
                        flag = flag + 1
                        loop_rate.sleep()
                    self.pose = None
                    print("发生大偏差，重定位成功")
                    rospy.sleep(0.5)
                else:
                    print("无需重定位")
                    pass
            else:
                print("新位置获取失败")
                pass  
        else:
            pass

    def master_go(self, goal):
        self.move_base_client.send_goal(goal)
        finished_within_time = self.move_base_client.wait_for_result(rospy.Duration(40))
        if not finished_within_time:
            self.move_base_client.cancel_goal()
            rospy.logerr("move_base超时")
        else:
            state = self.move_base_client.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("成功到达")
                return True
            else:
                rospy.logerr("没超时但失败")
                return False

    def actuator_deallocation(self,msg):
        if msg.name == 'Slave':
            self.slave_location=msg.status
            if self.slave_location == 'Fulfile' or self.slave_location == 'Init':
                self.fulfile_task = True
            else:
                self.fulfile_task = False

        

if  __name__ == "__main__":
    try:
        Master_Car = Car_go()
        machine = SimpleStateMachine(Master_Car)
        while not rospy.is_shutdown():
            machine.Go()

    except rospy.ROSInterruptException: 
        rospy.loginfo("运动失败")
