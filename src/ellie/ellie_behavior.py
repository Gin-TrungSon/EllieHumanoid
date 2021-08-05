import json
from abc import abstractmethod
from pathlib import Path
import sys
sys.path.append("")


class EllieBehavior:
    @abstractmethod
    def awake():
        pass

    @abstractmethod
    def start():
        pass

    @abstractmethod
    def update(self):
        pass

    @abstractmethod
    def last_update(self):
        pass

    @abstractmethod
    def on_exit(self):
        pass


class Behavior:
    def __init__(self, id, see="", listen="" , eyes_motion="", body_motion="", speak="", url=""):
        self.id = id
        self.behavior = {
            "id":id,
            "see": see,
            "listen": listen,
            "eyes_motion": eyes_motion,
            "body_motion": body_motion,
            "speak": speak,
            "url" : url
        }

    def save(self):     
        with open(f"src/ellie/behaviors/behaviors.json", "r") as write_file:
            parsed_json = json.load(write_file)
        with open(f"src/ellie/behaviors/behaviors.json", "w") as write_file:
            for i in parsed_json["intents"] :
                if i["id"] == self.id:
                    parsed_json["intents"].remove(i)
            parsed_json["intents"].append(self.behavior)
            json.dump(parsed_json,write_file,indent=6)

    def delete(self,id):     
        with open(f"src/ellie/behaviors/behaviors.json", "r") as write_file:
            parsed_json = json.load(write_file)
        with open(f"src/ellie/behaviors/behaviors.json", "w") as write_file:
            for i in parsed_json["intents"] :
                if i["id"] == id:
                    parsed_json["intents"].remove(i)
            json.dump(parsed_json,write_file,indent=6)
    @property
    def isFileExisted(self):
        return Path(f"src/ellie/behaviors/{self.id}.json").is_file()


class CameraSettings:
    pass

if __name__=="__main__":
    b = Behavior("l44", "Lady", "" , "eyes1", "body1", "speak1")
    b.save()
    #b.delete("l44")
