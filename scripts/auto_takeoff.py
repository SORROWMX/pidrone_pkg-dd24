#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import Bool, Empty
from geometry_msgs.msg import Pose
from pidrone_pkg.msg import Mode, State

class AutoTakeoff:
    def __init__(self):
        # Инициализация узла ROS
        rospy.init_node('auto_takeoff_node')
        
        # Параметры
        self.target_height = rospy.get_param('~target_height', 0.3)  # Целевая высота в метрах
        self.current_height = 0.0
        self.height_reached = False
        self.height_tolerance = 0.05  # Допустимое отклонение от целевой высоты (±5 см)
        
        # Издатели (Publishers)
        self.mode_pub = rospy.Publisher('/pidrone/desired/mode', Mode, queue_size=1)
        self.position_control_pub = rospy.Publisher('/pidrone/position_control', Bool, queue_size=1)
        self.pose_pub = rospy.Publisher('/pidrone/desired/pose', Pose, queue_size=1)
        
        # Подписчики (Subscribers)
        rospy.Subscriber('/pidrone/state/ukf_7d', State, self.state_callback)
        
        # Сообщения
        self.mode_msg = Mode()
        self.pose_msg = Pose()
        
        rospy.loginfo("Auto Takeoff node initialized. Target height: %.2f m", self.target_height)
        
    def state_callback(self, msg):
        # Получение текущей высоты дрона
        self.current_height = msg.pose_with_covariance.pose.position.z
        
        # Проверка достижения целевой высоты
        if abs(self.current_height - self.target_height) <= self.height_tolerance and not self.height_reached:
            self.height_reached = True
            rospy.loginfo("Target height reached: %.2f m. Switching to Position Hold mode.", self.current_height)
            self.enable_position_hold()
    
    def takeoff(self):
        # Подождать, чтобы убедиться, что все соединения установлены
        rospy.sleep(1.0)
        
        # Переключение в режим полета (FLYING)
        self.mode_msg.mode = "FLYING"
        self.mode_pub.publish(self.mode_msg)
        rospy.loginfo("Takeoff initiated")
        
        # Подождать, пока дрон не достигнет целевой высоты или пока не истечет таймаут
        timeout = rospy.Time.now() + rospy.Duration(30)  # 30 секунд таймаут
        rate = rospy.Rate(10)  # 10 Гц
        
        while not self.height_reached and rospy.Time.now() < timeout and not rospy.is_shutdown():
            # Постепенно увеличиваем высоту для плавного взлета
            current_target = min(self.target_height, self.current_height + 0.05)
            
            # Устанавливаем желаемую позицию для взлета
            self.pose_msg.position.x = 0
            self.pose_msg.position.y = 0
            self.pose_msg.position.z = current_target
            self.pose_pub.publish(self.pose_msg)
            
            rate.sleep()
        
        if self.height_reached:
            rospy.loginfo("Takeoff completed successfully")
        else:
            rospy.logwarn("Takeoff timeout or interrupted")
    
    def enable_position_hold(self):
        # Включение режима удержания позиции
        position_control_msg = Bool()
        position_control_msg.data = True
        self.position_control_pub.publish(position_control_msg)
        
        # Установка текущей позиции как целевой
        self.pose_msg.position.x = 0
        self.pose_msg.position.y = 0
        self.pose_msg.position.z = self.target_height
        self.pose_pub.publish(self.pose_msg)
        
        rospy.loginfo("Position Hold mode enabled at height %.2f m", self.target_height)

def main():
    auto_takeoff = AutoTakeoff()
    auto_takeoff.takeoff()
    
    # Держать узел активным
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass 