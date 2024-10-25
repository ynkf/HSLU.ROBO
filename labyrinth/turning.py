class Turning:
    
    # Define possible turning directions
    turning_rules = {
        ('RIGHT', 'UP'): ('LEFT'),
        ('UP', 'LEFT'): ('LEFT'),
        ('LEFT', 'DOWN'): ('LEFT'),
        ('DOWN', 'RIGHT'): ('LEFT'),
        
        ('UP', 'RIGHT'): ('RIGHT'),
        ('LEFT', 'UP'): ('RIGHT'),
        ('DOWN', 'LEFT'): ('RIGHT'),
        ('RIGHT', 'DOWN'): ('RIGHT'),
    }
    
    def __init__(self, robot_name):
        rospy.init_node('Turning', anonymous=True)
    
    def turning(self, current_direction, next_direction):
        if current_direction != next_direction:
            turn_direction = turning_rule.get(current_direction, next_direction)
            
            
            
            turn_nodes.append((i, turn_type, turn_angle))
            print(f'Node: {i} is a turn node! Turn: {turn_type} by {turn_angle}°')