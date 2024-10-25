import rospy
from wheel_command_publisher import WheelCommandPublisher

class Turning:
    
    def __init__(self, robot_name):
        # das bruchts glaub ned? mümmer nur einisch im navigator mache
        rospy.init_node('Turning', anonymous=True)
        self.wheel_command_publisher = WheelCommandPublisher(robot_name)
        self.turning_rules = {
        ('RIGHT', 'UP'): ('LEFT'),
        ('UP', 'LEFT'): ('LEFT'),
        ('LEFT', 'DOWN'): ('LEFT'),
        ('DOWN', 'RIGHT'): ('LEFT'),
        
        ('UP', 'RIGHT'): ('RIGHT'),
        ('LEFT', 'UP'): ('RIGHT'),
        ('DOWN', 'LEFT'): ('RIGHT'),
        ('RIGHT', 'DOWN'): ('RIGHT'),
        }
    
    def turning_direction(self, current_direction, next_direction):
        return self.turning_rules.get(current_direction, next_direction)
    
    def turn(self, turning_direction):
        if turning_direction == 'LEFT':
            self.wheel_command_publisher.turn_wheels(-0.1, 0.1)
        elif turning_direction == 'RIGHT':
            self.wheel_command_publisher.turn_wheels(0.1, -0.1)
        rospy.sleep(1) # TEST! -> should be 90°
        self.wheel_command_publisher.stop_wheels()