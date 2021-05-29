import sys
sys.path.append("")
from src.ellie.ellie_body.ellie_body_ultis import *
from src.ellie.ellie_behavior import EllieBehavior
class Motors:
    """
    0:      l_elbow_y
    1:      head_y 
    2:      r_arm_z 
    3:      head_z 
    4:      r_shoulder_x
    5:      r_shoulder_y
    6:      r_elbow_y
    7:      l_arm_z
    8:      abs_z
    9:      bust_y
    10:     bust_x
    11:     l_shoulder_x
    12:     l_shoulder_y
"""
    def __init__(self,poppy):
        self.l_elbow_y= poppy.motors[0]
        self.head_y = poppy.motors[1]
        self.r_arm_z = poppy.motors[2]
        self.head_z = poppy.motors[3]
        self.r_shoulder_x= poppy.motors[4]
        self.r_shoulder_y= poppy.motors[5]
        self.r_elbow_y= poppy.motors[6]
        self.l_arm_z= poppy.motors[7]
        self.abs_z= poppy.motors[8]
        self.bust_y= poppy.motors[9]
        self.bust_x= poppy.motors[10]
        self.l_shoulder_x= poppy.motors[11]
        self.l_shoulder_y= poppy.motors[12]
    
    def goto_position(self,motor,amp,wait_time,wait =False , freq=0.5):
        if wait == False:
            self._step_calculate(motor,amp,wait_time,freq)
        else:
            thread = threading.Thread(target=self._step_calculate(motor,amp,wait_time,freq))
            thread.start()

    def _step_calculate(self,motor, amp,wait_time, freq):
        t0= time.time()
        while True:
            t= time.time()
            if t-t0 > wait_time:
                break
            motor.goal_position=  amp * math.sin(2 * 3.14 * freq * t)
            time.sleep(0.01)


class EllieBody(EllieBehavior):
    def __init__(self, simulator= None) -> None:
        self.robot = PoppyTorso(simulator="vrep")
        # with open("test.text","a" ) as f:
        #      f.write(str(self.robot.to_config()))
        #self.robot = from_json("src/body/config/torso.json")
        self.setup()
        super.__init__()
    def attach_primitives(self, isCamera=True):

        # loop to attach all .move config file 
        for file in glob.glob("src/body/actions/*.move"):
            print(file)
            move_name = os.path.splitext(file)[0]
            self.robot.attach_primitive(EllieAction(self.robot,action=move_name),move_name)


        # Attach Perpetual Movement
        self.robot.attach_primitive(UpperBodyIdleMotion(self.robot, 50), 'upper_body_idle_motion')
        self.robot.attach_primitive(HeadIdleMotion(self.robot, 50), 'head_idle_motion')
        self.robot.attach_primitive(TorsoIdleMotion(self.robot, 50), 'torso_idle_motion')

        self.robot.attach_primitive(Rest(self.robot), 'rest')
        self.robot.attach_primitive(Off(self.robot), 'off')
    def setup(self):
        
        with open("src/body/config/conf.json") as data:
            ellie_cfg = json.load(data)
        port = ellie_cfg["robot"]["port"]
        name = ellie_cfg["robot"]["name"]
        
        for motor in self.robot.motors:
            motor.compliant_behavior ="dummy"
            motor.goto_beahavior = "minjerk"
            motor.moving_speed = 80

        for motor in self.robot.motors:
            motor.compliant = False
            motor.goal_position =0
            print(motor)

        for motor in self.robot.head:
            motor.complant = True
        try:
            self.attach_primitives(self.robot)
        except:
            print("Primitives not attached ")
        else:
            print("Primitives attached successfull")

    def update(self, context):
        pass
    def learn(self):
        move = MoveRecorder(self.robot,100, self.robot.motors)
        self.roobt.compliant = True

        input("Press enter to start recording")

        for x in range(3,0,-1):
            print(x)
            time.sleep()
        
        move.start()
        input("Press enter to start recording")
        move.stop()
    
        for m in self.robot.motors:
            m.compliant = False
            m.goal_position = 0

        move_name = input("Enter the name of this sick move : ")
        move_name = move_name+".move"

        with open("src/body/actions/"+move_name, 'w') as f:
            try:
                move.move.save(f)
            except:
                print("Unable to save this motion")
            else:
                print("Move successfully saved !")
        try:
            self.robot.attach_primitive(EllieAction(self.robot,movement=move_name),move_name)
        except Exception as e:
            raise
        else:
            print("Move successfully attached to the robot !")
    
    def test_motion(self):
        while True:
            input("start")
            p = get_primitives()
            for i in p:
                m_path = i
                print(m_path)
                with open(m_path+".move") as f:
                    m = Move.load(f) # chargement du .move
                move = MovePlayer(self.robot,play_speed=0.1,move= m)
                move.start()
                move.wait_to_stop()
    @property
    def greet(self):
        with open("src/body/actions/behave_handsup.move") as f:
                m = Move.load(f) # chargement du .move
                move = MovePlayer(self.robot,play_speed=0.2,move= m)
                move.start()
                move.wait_to_stop()

    def test_greet(self):
        input("start")
        low_down =10
        input("continue")



if __name__=="__main__":
    ellie = EllieBody(simulator="vrep")
    ellie.greet
