from abc import ABC, abstractmethod

class Player(ABC):
    
    @abstractmethod
    def guess_object(self, spy_text: str) -> str:
        pass