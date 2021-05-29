import pypot.primitive
import pypot.primitive

class Off(pypot.primitive.Primitive):

	def run(self):
		try:
			self.robot.upper_body_idle_motion.stop()
			self.robot.torso_idle_motion.stop()
		except:
			pass

		for m in self.robot.motors:
			m.compliant = True