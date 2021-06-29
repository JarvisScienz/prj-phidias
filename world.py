#
#
#
import math

from block import *

class World:

    FLOOR_LEVEL = -0.05

    def __init__(self, ui):
        self.__blocks = [ ]
        self.ui = ui

    def new_block(self, uColor, uX):
        b = Block(uColor)
        b.set_pose(uX, World.FLOOR_LEVEL, 0)
        self.__blocks.append(b)

    def new_obstacle(self, uX, uY):
        obstacle = Block('black')
        obstacle.set_pose(uX, uY, 0)
        self.__blocks.append(obstacle)

    def count_blocks(self):
        return len(self.__blocks)

    def floor_position_busy(self, uX):
        for b in self.__blocks:
            (x,y,a) = b.get_pose()
            if (uX >= x)and(uX <= (x + Block.WIDTH)):
                return True
        return False

    def sense_distance(self):
        (x,y,a) = self.ui.arm.get_pose_xy_a().get_pose()
        a = math.degrees(a)
        if abs(a + 90) > 2: # +/- 2 degrees
            return None
        L = self.ui.arm.element_3_model.L
        d = y - L - World.FLOOR_LEVEL
        for b in self.__blocks:
            (xb,yb,ab) = b.get_pose()
            if (x >= xb)and(x <= (xb + Block.WIDTH)):
                return (d - Block.HEIGHT)
        return None

    def sense_color(self):
        (x,y,a) = self.ui.arm.get_pose_xy_a().get_pose()
        a = math.degrees(a)
        if abs(a + 90) > 2: # +/- 2 degrees
            return None
        L = self.ui.arm.element_3_model.L
        d = y - L - World.FLOOR_LEVEL
        for b in self.__blocks:
            (xb,yb,ab) = b.get_pose()
            if (x >= xb)and(x <= (xb + Block.WIDTH)):
                return b.get_color()
        return None

    def paint(self, qp):
        for b in self.__blocks:
            b.paint(qp)
        qp.setPen(QColor(217,95,14))
        y = Pose.xy_to_pixel(0, World.FLOOR_LEVEL)[1]
        qp.drawLine(50, y, 900, y)
        qp.drawLine(50, y+1, 900, y+1)

