
import math


class Trajectory:

    def __init__(self, vmax, accel, decel):
        self.vmax = vmax
        self.accel = accel
        self.decel = decel
        self.target_pos = 0
        self.target_heading = 0
        self.virtual_pos = 0
        self.virtual_speed = 0
        self.decel_distance = self.vmax * self.vmax / (2 * self.decel)
        self.accel_distance = self.vmax * self.vmax / (2 * self.accel)

    def set_target(self, current_x, current_y, target_x, target_y):
        self.virtual_pos = 0
        self.virtual_speed = 0
        dx = target_x - current_x
        dy = target_y - current_y

        self.target_heading = math.atan2(dy, dx)
        self.target_pos = math.hypot(dx, dy)
        self.start_x = current_x
        self.start_y = current_y


    def evaluate(self, delta_t):
        distance = self.target_pos - self.virtual_pos

        if distance < self.decel_distance:
            arg = self.vmax * self.vmax - 2 * self.decel * (self.decel_distance - distance)
            if arg < 0:
                current_accel = 0
                self.virtual_speed = 0
            else:
                vel_attesa = math.sqrt(arg)
                if vel_attesa > self.virtual_speed:
                    # siamo ancora in accelerazione
                    current_accel = self.accel
                else:
                    # fase di decelerazione
                    current_accel = -self.decel
        else:
            # fase di accelerazione o moto a vel costante
            current_accel = self.accel

        self.virtual_pos += self.virtual_speed * delta_t + \
          0.5 * current_accel * delta_t * delta_t

        self.virtual_speed += current_accel * delta_t

        if self.virtual_speed >= self.vmax:
            self.virtual_speed = self.vmax

        if self.virtual_speed <= 0:
            self.virtual_speed = 0

        vp_x = self.virtual_pos * math.cos(self.target_heading)
        vp_y = self.virtual_pos * math.sin(self.target_heading)
        return (self.start_x + vp_x, self.start_y + vp_y)




class Trajectory3:

    def __init__(self, vmax, accel, decel):
        self.vmax = vmax
        self.accel = accel
        self.decel = decel
        self.target_pos = 0
        self.target_heading = 0
        self.virtual_pos = 0
        self.virtual_speed = 0
        self.decel_distance = self.vmax * self.vmax / (2 * self.decel)
        self.accel_distance = self.vmax * self.vmax / (2 * self.accel)
        self.arm = None
        self.target_got = False

    def set_target(self, current_x, current_y, current_alpha, target_x, target_y, target_alpha):
        self.virtual_pos = 0
        self.virtual_speed = 0
        dx = target_x - current_x
        dy = target_y - current_y
        dz = target_alpha - current_alpha

        self.target_alpha = target_alpha

        # sferic coordinates
        self.target_pos = math.sqrt(dx**2 + dy**2 + dz**2)
        self.target_theta = math.atan2(math.hypot(dx,dy), dz)
        self.target_phi = math.atan2(dy, dx)

        self.start_x = current_x
        self.start_y = current_y
        self.start_alpha = current_alpha

        self.target_x = target_x
        self.target_y = target_y
        self.target_alpha = target_alpha

        self.target_got = False
        self.target_count = 0

        #print('------------------------------------------------')
        #print(self.start_x, self.start_y, self.start_alpha)
        #print(target_x, target_y, target_alpha)
        #print(self.target_pos, math.degrees(self.target_theta), math.degrees(self.target_phi))
        #import time
        #time.sleep(1)


    def evaluate(self, delta_t):
        distance = self.target_pos - self.virtual_pos

        if distance < self.decel_distance:
            arg = self.vmax * self.vmax - 2 * self.decel * (self.decel_distance - distance)
            if arg < 0:
                current_accel = 0
                self.virtual_speed = 0
            else:
                vel_attesa = math.sqrt(arg)
                if vel_attesa > self.virtual_speed:
                    # siamo ancora in accelerazione
                    current_accel = self.accel
                else:
                    # fase di decelerazione
                    current_accel = -self.decel
        else:
            # fase di accelerazione o moto a vel costante
            current_accel = self.accel

        self.virtual_pos += self.virtual_speed * delta_t + \
          0.5 * current_accel * delta_t * delta_t

        self.virtual_speed += current_accel * delta_t

        if self.virtual_speed >= self.vmax:
            self.virtual_speed = self.vmax

        if self.virtual_speed <= 0:
            self.virtual_speed = 0

        vp_x = self.start_x + self.virtual_pos * math.cos(self.target_phi) * math.sin(self.target_theta)
        vp_y = self.start_y + self.virtual_pos * math.sin(self.target_phi) * math.sin(self.target_theta)
        vp_a = self.start_alpha + self.virtual_pos * math.cos(self.target_theta)
        #print(vp_x, vp_y, vp_a, distance)

        (cx, cy, ca) = self.arm.get_pose_xy_a().get_pose()

        d = math.sqrt( (cx-self.target_x)**2 + (cy-self.target_y)**2 + (ca-self.target_alpha)**2)

        if d < 1e-2:
            self.target_count += 1
            if self.target_count > 10:
                self.target_got = True
        else:
            self.target_count = 0

        return (vp_x, vp_y, vp_a)



