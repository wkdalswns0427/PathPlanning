from local_planner.utils.imports import *
show_animation = True

class State:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.rear_x = self.x - ((WB / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((WB / 2) * math.sin(self.yaw))

    def update(self, a, delta):
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw += self.v / WB * math.tan(delta) * dt
        self.v += a * dt
        self.rear_x = self.x - ((WB / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((WB / 2) * math.sin(self.yaw))

    def calc_distance(self, point_x, point_y):
        dx = self.rear_x - point_x
        dy = self.rear_y - point_y
        return math.hypot(dx, dy)


class States:
    def __init__(self):
        self.x = []
        self.y = []
        self.yaw = []
        self.v = []
        self.t = []

    def append(self, t, state):
        self.x.append(state.x)
        self.y.append(state.y)
        self.yaw.append(state.yaw)
        self.v.append(state.v)
        self.t.append(t)


class TargetCourse:
    # cx, cy : global path
    def __init__(self, cx, cy):
        self.cx = cx
        self.cy = cy
        self.old_nearest_point_index = None

    def search_target_index(self, state):
        # To speed up nearest point search, doing it at only first time.
        if self.old_nearest_point_index is None:
            # search nearest point index
            dx = [state.rear_x - icx for icx in self.cx]
            dy = [state.rear_y - icy for icy in self.cy]
            d = np.hypot(dx, dy)
            ind = np.argmin(d)
            self.old_nearest_point_index = ind
        else:
            ind = self.old_nearest_point_index
            distance_this_index = state.calc_distance(self.cx[ind],
                                                      self.cy[ind])
            while True:
                distance_next_index = state.calc_distance(self.cx[ind + 1],
                                                          self.cy[ind + 1])
                if distance_this_index < distance_next_index:
                    break
                ind = ind + 1 if (ind + 1) < len(self.cx) else ind
                distance_this_index = distance_next_index
            self.old_nearest_point_index = ind

        Lf = k * state.v + Lfc  # update look ahead distance

        # search look ahead target point index
        while Lf > state.calc_distance(self.cx[ind], self.cy[ind]):
            if (ind + 1) >= len(self.cx):
                break  # not exceed goal
            ind += 1

        return ind, Lf

"""
paramters
cx, cy : target path [np.array]
initial_pos : [x,y, yaw, v]
"""
class PurePursuit:
    def __init__(self) -> None:
        global k, Lfc, Kp, dt, WB
        with open("/home/mjc/PathPlanning/config/local_path.yaml") as f:
            planner_config = yaml.load(f, Loader=yaml.FullLoader)
        k = planner_config["parameters"]["k"]  # look forward gain
        Lfc = planner_config["parameters"]["Lfc"]  # [m] look-ahead distance
        Kp = planner_config["parameters"]["Kp"]  # speed proportional gain
        dt = planner_config["parameters"]["dt"]  # [s] time tick
        WB = planner_config["parameters"]["WB"]  # [m] wheel base of vehicle
    
    def proportional_control(self, target, current):
        a = Kp * (target - current)
        return a

    def pure_pursuit_steer_control(self, state, trajectory, pind):
        ind, Lf = trajectory.search_target_index(state)

        if pind >= ind:
            ind = pind

        if ind < len(trajectory.cx):
            tx = trajectory.cx[ind]
            ty = trajectory.cy[ind]
        else:  # toward goal
            tx = trajectory.cx[-1]
            ty = trajectory.cy[-1]
            ind = len(trajectory.cx) - 1

        alpha = math.atan2(ty - state.rear_y, tx - state.rear_x) - state.yaw
        delta = math.atan2(2.0 * WB * math.sin(alpha) / Lf, 1.0)

        return delta, ind

    # unnecessary
    def plot_arrow(self, x, y, yaw, length=1.0, width=0.5, fc="r", ec="k"):
        if not isinstance(x, float):
            for ix, iy, iyaw in zip(x, y, yaw):
                self.plot_arrow(ix, iy, iyaw)
        else:
            plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
                    fc=fc, ec=ec, head_width=width, head_length=width)
            plt.plot(x, y)

    def pure_pursuit(self, cx, cy, initial_pos):
        target_speed = 10.0 / 3.6  # [m/s]
        T = 100.0  # max simulation time

        # initial state
        state = State(x=initial_pos[0], y=initial_pos[1], yaw=initial_pos[2], v=initial_pos[3])

        lastIndex = len(cx) - 1
        time = 0.0
        states = States()
        states.append(time, state)
        target_course = TargetCourse(cx, cy)
        target_ind, _ = target_course.search_target_index(state)
        L = math.sqrt((cx[target_ind] - cx[1])**2 + (cy[target_ind] - cy[1])**2)
        theta = math.atan2((cy[target_ind] - cy[1]),(cx[target_ind] - cx[1]))
    
        while T >= time and lastIndex > target_ind:
            # Calc control input
            ai = self.proportional_control(target_speed, state.v)
            di, target_ind = self.pure_pursuit_steer_control(
                state, target_course, target_ind)

            state.update(ai, di)  # Control vehicle
            time += dt
            states.append(time, state)
            theta = math.atan2((cy[target_ind] - cy[target_ind-1]),(cx[target_ind] - cx[target_ind-1]))
            L = math.sqrt((cx[target_ind] - cx[target_ind-1])**2 + (cy[target_ind] - cy[target_ind-1])**2)

            # if show_animation:  # pragma: no cover
            #     plt.cla()
            #     # for stopping simulation with the esc key.
            #     plt.gcf().canvas.mpl_connect(
            #         'key_release_event',
            #         lambda event: [exit(0) if event.key == 'escape' else None])
            #     self.plot_arrow(state.x, state.y, state.yaw)
            #     plt.plot(cx, cy, "-r", label="course")
            #     plt.plot(states.x, states.y, "*b", label="trajectory")
            #     plt.plot(cx[target_ind], cy[target_ind], "xg", label="target")
            #     plt.axis("equal")
            #     plt.grid(True)
            #     plt.title("Speed[km/h]:" + str(state.v * 3.6)[:4])
            #     plt.pause(0.001)
        print(f"RETURN : {[states.v[1]* 3.6, theta*states.v[1]* 3.6/L]}")#*math.cos(theta)
        assert lastIndex >= target_ind, "Cannot goal"


        if show_animation:  # pragma: no cover
            plt.cla()
            plt.plot(cx, cy, ".r", label="course")
            plt.plot(states.x, states.y, "*b", label="trajectory")
            plt.legend()
            plt.xlabel("x[m]")
            plt.ylabel("y[m]")
            plt.axis("equal")
            plt.grid(True)

            plt.subplots(1)
            plt.plot(states.t, [iv * 3.6 for iv in states.v], "-r")
            plt.xlabel("Time[s]")
            plt.ylabel("Speed[km/h]")
            plt.grid(True)
            plt.show()
        
        return [states.v[1]* 3.6, theta*states.v[1]* 3.6/L]


if __name__ == '__main__':
    cx = np.arange(0, 50, 0.5)
    cy = [math.cos(ix / 5.0) * ix + math.sin(ix / 5.0) * ix / 2.0 for ix in cx]
    PP = PurePursuit()
    PP.pure_pursuit(cx,cy,[0.0,0.0,0.0,0.0])
