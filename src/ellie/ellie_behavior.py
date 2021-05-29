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
    def last_update():
        pass