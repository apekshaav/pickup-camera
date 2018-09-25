# SET DA VINVI ARM POSITIONS
#
# This script sets the initial positions of PSM2, PSM3, MTML, MTMR for use in the pickup camera study.
# Only joint angles are used.

import dvrk
import numpy as np

class Positions:

	def init(self):
		self.p2 = dvrk.psm('PSM2')
		self.p3 = dvrk.psm('PSM3')

		self.ml = dvrk.mtm('MTML')
		self.mr = dvrk.mtm('MTMR')

		# self.p2_pos = np.zeros(6)
		# self.p3_pos = np.zeros(6)
		# self.ml_pos = np.zeros(7)
		# self.mr_pos = np.zeros(7)

		self.count = 1;

	def getPos(self):
		self.p2_pos = self.p2.get_current_joint_position()
		self.p3_pos = self.p3.get_current_joint_position()
		
		self.ml_pos = self.ml.get_current_joint_position()
		self.mr_pos = self.mr.get_current_joint_position()

		print(self.p2_pos)
		# print(p3_pos)
		# print(ml_pos)
		# print(mr_pos)


	def savePos(self):
		print "savePos"
		# filename = "positions" + str(count) + ".txt"

		# f = open(filename, "w+")
		# f.write("PSM2: ")	

	
	def moveToPos(self):
		print(self.p2_pos)
		# print(p3_pos)
		# print(ml_pos)
		# print(mr_pos)

		self.p2.move_joint(self.p2_pos)
		# p3.move_joint(p3_pos)
	
		# ml.move_joint(ml_pos)
		# mr.move_joint(mr_pos)	


	def moveToPosFromFile(self):
		print "moveToPosFromFile"
		# filename = str(input("Enter file name: "))
		# f = open(filename, "r")


	def menu(self):
		print "\nOptions:"
		print "1. Get positions"
		print "2. Save positions to file"
		print "3. Move to position"
		print "4. Move to position from file"
		print "5. Exit\n"


	def selectFromMenu(self, argument):
		switcher = {
				1: self.getPos,
				2: self.savePos,
				3: self.moveToPos,
				4: self.moveToPosFromFile
			}
		func = switcher.get(argument, lambda: "Invalid option")
		func()


	def main(self):		
		option = 0
		while option != 5:
			self.menu()
			option = int(input("Enter your choice: "))
			self.selectFromMenu(option) 


if __name__ == "__main__":
	pos = Positions()
	pos.init()
	pos.main()