from abc import abstractmethod
class EllieBehavior:
    @abstractmethod
    def awake():
        pass
    @abstractmethod
    def start():
        pass
    @abstractmethod
    def update(self, context):
        pass
    @abstractmethod
    def last_update(self):
        pass
    
    @abstractmethod
    def on_exit(self):
        pass