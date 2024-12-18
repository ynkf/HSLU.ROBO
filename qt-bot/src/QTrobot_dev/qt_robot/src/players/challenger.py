from abc import ABC, abstractmethod

class Challenger(ABC):
    
    @abstractmethod
    def choose_spy_object(self) -> str:
        pass
    
    @abstractmethod
    def check_spy_object(self, guess: str):
        pass