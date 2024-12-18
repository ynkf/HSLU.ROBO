import random



from object_detection import objectDetection
from QTrobot_dev.qt_robot.src.players.challenger import Challenger
from player import Player

class QtBot(Challenger, Player):
    
    def __init__(self):
        self.object_detection = objectDetection()
        
        self.spy_object = None

    
    def choose_spy_object(self) -> str:
        objects = self.object_detection.detect_objects()
        object_list = [object for object in objects if object.class_name in self.object_whitelist]
            
        object_set = set(map(lambda x: x.class_name, object_list))
        self.spy_object = random.choice(list(object_set))
    
        i_spy_text = f'I spy something starting with the letter {self.spy_object[0]}'
        self.talk_text_service(i_spy_text)
        
            
    def check_spy_object(self, guess):
        # TODO: compare if the guess corresponds to self.spy_object
        pass
    
    
    def guess_object(self, spy_text: str) -> str:
        objects = self.object_detection.detect_objects()
        object_list = [object for object in objects if object.class_name in self.object_whitelist]
        
        # TODO: use list of objects and add the initial text to the ai prompt to guess a object
        pass