import pypot.primitive

class Rest(pypot.primitive.Primitive):
	def run(self):
		try:
			self.robot.upper_body_idle_motion.stop()
			self.robot.torso_idle_motion.stop()
		except:
			pass

		for m in self.robot.motors:
			m.compliant = False
			m.goal_position = 0

		for m in self.robot.head:
			m.compliant = True
