import rospy
from pidrone_pkg.msg import Mode
import getch
import time
import os
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose


def main():
    node_name = os.path.splitext(os.path.basename(__file__))[0]
    rospy.init_node(node_name)
    import sys, tty, termios
    fd = sys.stdin.fileno()
    #attr = termios.tcgetattr(sys.stdin.fileno())
    #tty.setraw(sys.stdin.fileno())
    mode = Mode()
    mode.mode = 4
    modepub = rospy.Publisher('/pidrone/set_mode', Mode, queue_size=1)
    
    # Добавляем издатели для управления позицией
    positionPub = rospy.Publisher('/pidrone/position_control', Bool, queue_size=1)
    posePub = rospy.Publisher('/pidrone/desired/pose', Pose, queue_size=1)
    
    # Параметры для автоматического взлета
    target_height = 0.3  # целевая высота в метрах
    
    rate = rospy.Rate(10)
    msg = """
Commands:
;:  arm
' ' (spacebar):  disarm
t:  takeoff
T:  takeoff to height and hold position
l:  land
a:  yaw left
d:  yaw right
w:  up
z:  down
i:  forward
k:  backward
j:  left
l:  right
h:  hover
q:  quit
"""
    try:
        print(msg)
        while not rospy.is_shutdown():
            ch = getch.getch(0.1)
            if ch == None:
                continue

            if ord(ch) == 32:
                # disarm
                print("disarming")
                mode.mode = 4
                mode.x_velocity = 0
                mode.y_velocity = 0
                mode.z_velocity = 0
                mode.yaw_velocity = 0
                modepub.publish(mode)
            elif ch == ";":
                # arm
                print("arm")
                mode.mode = 0
                mode.x_velocity = 0
                mode.y_velocity = 0
                mode.z_velocity = 0
                mode.yaw_velocity = 0
                modepub.publish(mode)
            elif ch == "y":
                # land
                print("land")
                mode.x_velocity = 0
                mode.y_velocity = 0
                mode.z_velocity = 0
                mode.yaw_velocity = 0
                mode.mode = 3
                modepub.publish(mode)
            elif ch == "h":
                # hover
                print("hover")
                mode.mode = 5
                mode.x_velocity = 0
                mode.y_velocity = 0
                mode.z_velocity = 0
                mode.yaw_velocity = 0
                modepub.publish(mode)
            elif ch == "t":
                print("takeoff")
                mode.x_velocity = 0
                mode.y_velocity = 0
                mode.z_velocity = 0
                mode.yaw_velocity = 0
                mode.mode = 5
                modepub.publish(mode)
            elif ch == "T":
                # Взлет на определенную высоту с последующим удержанием позиции
                print(f"takeoff to height {target_height}m and hold position")
                # Сначала взлетаем
                mode.x_velocity = 0
                mode.y_velocity = 0
                mode.z_velocity = 0
                mode.yaw_velocity = 0
                mode.mode = 5
                modepub.publish(mode)
                
                # Даем время на взлет (примерно 3 секунды)
                time.sleep(3)
                
                # Затем включаем режим удержания позиции
                position_control_msg = Bool()
                position_control_msg.data = True
                positionPub.publish(position_control_msg)
                
                # Устанавливаем желаемую позицию
                pose_msg = Pose()
                pose_msg.position.x = 0
                pose_msg.position.y = 0
                pose_msg.position.z = target_height
                posePub.publish(pose_msg)
                print(f"Position hold enabled at height {target_height}m")
            elif ch == "j":
                print("left")
                mode.mode = 5
                mode.x_velocity = -3
                mode.y_velocity = 0
                mode.z_velocity = 0
                mode.yaw_velocity = 0
                modepub.publish(mode)
            elif ch == "l":
                print("right")
                mode.mode = 5
                mode.x_velocity = 3
                mode.y_velocity = 0
                mode.z_velocity = 0
                mode.yaw_velocity = 0
                modepub.publish(mode)
            elif ch == "i":
                print("forward")
                mode.mode = 5
                mode.x_velocity = 0
                mode.y_velocity = 3
                mode.z_velocity = 0
                mode.yaw_velocity = 0
                modepub.publish(mode)
            elif ch == "k":
                print("backward")
                mode.mode = 5
                mode.x_velocity = 0
                mode.y_velocity = -3
                mode.z_velocity = 0
                mode.yaw_velocity = 0
                modepub.publish(mode)
            elif ch == "w":
                print("up")
                mode.mode = 5
                mode.x_velocity = 0
                mode.y_velocity = 0
                mode.z_velocity = 3
                mode.yaw_velocity = 0
                modepub.publish(mode)
            elif ch == "z":
                print("down")
                mode.mode = 5
                mode.x_velocity = 0
                mode.y_velocity = 0
                mode.z_velocity = -3
                mode.yaw_velocity = 0
                modepub.publish(mode)
            elif ch == "a":
                print("yaw left")
                mode.mode = 5
                mode.x_velocity = 0
                mode.y_velocity = 0
                mode.z_velocity = 0
                mode.yaw_velocity = -200
                modepub.publish(mode)
            elif ch == "d":
                print("yaw right")
                mode.mode = 5
                mode.x_velocity = 0
                mode.y_velocity = 0
                mode.z_velocity = 0
                mode.yaw_velocity = 200
                modepub.publish(mode)
            elif ch == "q":
                print("quit")
                mode.mode = 4
                modepub.publish(mode)
                break
            else:
                print("key not recognized")
    except Exception as e:
        print(e)
    finally:
        mode.mode = 4
        modepub.publish(mode)


if __name__ == "__main__":
    main()
