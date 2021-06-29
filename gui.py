#
#
#

import sys
import math
import random

#
from PyQt5.QtWidgets import QMainWindow, QWidget, QApplication
from PyQt5.QtGui import QPainter, QColor, QFont, QPixmap, QTransform
from PyQt5.QtCore import Qt, QTimer
# ^^^^^^^^^^^^^^^^^^^^^^^^^^^ use these for Qt5
#
#

#from PyQt4 import QtGui, QtCore

from arm import *
from arm_painters import *
from telemetry import *
from world import *
from phidias_interface import *

COLOR_NAMES = [ 'red',
                'green',                
                'blue' ]

POSITIONS = [ 0.04, 0.06, 0.08, 0.1, 0.12, 0.14, 0.16, 0.18, 0.2, 0.22 ]
OBSTACLE_POSITIONS_X = [ 0.04, 0.14, 0.08 ]
OBSTACLE_POSITIONS_Y = [ 0.02, -0.01, 0.02 ]


class MainWindow(QWidget):

    def __init__(self):
        super(MainWindow, self).__init__()

        self.initUI()

    def initUI(self):
        self.setGeometry(0, 0, 1000, 600)
        self.setWindowTitle('Robotic Arm Simulator')
        self.show()

        self.delta_t = 1e-2 # 10ms of time-tick
        self.t = 0

        self.trajectory_data = [ ]
        self.target_trajectory_data = [ ]

        self.use_profile = False
        self.use_trajectory = True
        self.use_angle_interpolation = False

        self.trajectory = Trajectory3(1.0, 1.5, 1.5)

        self.arm = ThreeJointsArm(self.trajectory, self.use_profile)
        self.painter = ThreeJointsArmPainter(self.arm)

        target_x = 0.1
        target_y = 0.05
        target_alpha = -90

        self.arm.set_target_xy_a(target_x, target_y, target_alpha)

        self.world = World(self)

        self.telemetry = Telemetry()

        self._timer_painter = QTimer(self)
        self._timer_painter.timeout.connect(self.go)
        self._timer_painter.start(self.delta_t * 1000)
        self.notification = False

        self._timer_sensor = QTimer(self)
        self._timer_sensor.timeout.connect(self.send_pos)
        self._timer_sensor.start(500)

        self._from = None

        #set obstacles
        self.generate_obstacles()

    def set_from(self, _from):
        self._from = _from

    def go_to(self,target_x, target_y, target_alpha):
        self.notification = False
        self.arm.set_target_xy_a(target_x, target_y, target_alpha)

    def generate_new_block(self):
        if self.world.count_blocks() == 6:
            return
        while True:
            #x = int(random.uniform(1, 9)) * (Block.WIDTH + Block.GAP)
            x = int(random.uniform(0, 10))
            pos = POSITIONS[x]
            col = int(random.uniform(0, 3))
            if not(self.world.floor_position_busy(pos)):
                #print("Position block: ", pos)
                self.world.new_block(COLOR_NAMES[col], pos)
                return

    def generate_obstacles(self):
        for i in range (3):
            posX = OBSTACLE_POSITIONS_X[i]
            posY = OBSTACLE_POSITIONS_Y[i]
            self.world.new_obstacle(posX, posY)

    def notify_target_got(self):
        self.notification = True
        if self._from is not None:
            Messaging.send_belief(self._from, 'target_got', [], 'robot')

    def send_pos(self):
        if self._from is not None:
            p = self.arm.get_pose_xy_a().get_pose()
            Messaging.send_belief(self._from, 'pose', [p[0], p[1], math.degrees(p[2])], 'robot')

    def sense_distance(self):
        if self._from is not None:
            d = self.world.sense_distance()
            if d is None:
                params = []
            else:
                params = [d]
            Messaging.send_belief(self._from, 'distance', params, 'robot')

    def sense_color(self):
        if self._from is not None:
            d = self.world.sense_color()
            if d is None:
                params = []
            else:
                params = [d]
            Messaging.send_belief(self._from, 'color', params, 'robot')

    def go(self):
        #self.telemetry.gather(self.delta_t, self.arm.element_3_model.w, self.arm.element_3_control.w_target)
        #if self.t > 8:
        #    self.telemetry.show()
        #    sys.exit(0)

        if self.trajectory.target_got:
            if not(self.notification):
                self.notify_target_got()

        self.arm.evaluate_trajectory(self.delta_t)

        #p = self.arm.get_pose()
        #self.trajectory_data.append(p[-1])
        #if self.use_trajectory:
        #    self.target_trajectory_data.append( (self.arm.trajectory_x, self.arm.trajectory_y) )
        self.t += self.delta_t
        self.update() # repaint window


    def paintEvent(self, event):
        qp = QPainter()
        qp.begin(self)
        qp.setPen(QColor(255,255,255))
        qp.setBrush(QColor(255,255,255))
        qp.drawRect(event.rect())

        qp.setPen(Qt.black)
        qp.drawLine(50, 500, 900, 500)
        qp.drawLine(50, 500, 50, 50)
        qp.drawLine(50, 50, 900, 50)
        qp.drawLine(900, 50, 900, 500)

        qp.setPen(Qt.black)
        self.painter.paint(qp, self.t)
        self.world.paint(qp)

        qp.end()



def main():

    app = QApplication(sys.argv)
    ex = MainWindow()
    start_message_server_http(ex)
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()

