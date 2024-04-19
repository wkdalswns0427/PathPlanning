from global_planner.utils.imports import *
from global_planner.utils.config import *
# from utils.imports import *
# from utils.config import *

class AStarPlanner:

    def __init__(self, ox, oy, resolution, rr):
        """
        Initialize grid map for a star planning
        ox: x position list of Obstacles [m]
        oy: y position list of Obstacles [m]
        resolution: grid resolution [m]
        rr: robot radius[m]
        """
        self.resolution = resolution
        self.rr = rr
        self.min_x, self.min_y = 0, 0
        self.max_x, self.max_y = 0, 0
        self.obstacle_map = None
        self.x_width, self.y_width = 0, 0
        self.motion = self.get_motion_model()
        self.calc_obstacle_map(ox, oy)

    class Node:
        def __init__(self, x, y, cost, parent_index):
            self.x = x  # index of grid
            self.y = y  # index of grid
            self.cost = cost
            self.parent_index = parent_index

        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(
                self.cost) + "," + str(self.parent_index)

    def planning(self, sx, sy, gx, gy):
        res = True
        """
        A star path search

        input:
            s_x: start x position [m]
            s_y: start y position [m]
            gx: goal x position [m]
            gy: goal y position [m]

        output:
            rx: x position list of the final path
            ry: y position list of the final path
        """

        start_node = self.Node(self.calc_xy_index(sx, self.min_x),
                               self.calc_xy_index(sy, self.min_y), 0.0, -1)
        goal_node = self.Node(self.calc_xy_index(gx, self.min_x),
                              self.calc_xy_index(gy, self.min_y), 0.0, -1)

        open_set, closed_set = dict(), dict()
        open_set[self.calc_grid_index(start_node)] = start_node

        while True:
            if len(open_set) == 0:
                print("Open set is empty..")
                res = False
                break

            c_id = min(
                open_set,
                key=lambda o: open_set[o].cost + self.calc_heuristic(goal_node,
                                                                     open_set[
                                                                         o]))
            current = open_set[c_id]

            # # show graph
            if show_animation:  # pragma: no cover
                plt.plot(self.calc_grid_position(current.x, self.min_x),
                         self.calc_grid_position(current.y, self.min_y), "xc")
                # for stopping simulation with the esc key.
                plt.gcf().canvas.mpl_connect('key_release_event',
                                             lambda event: [exit(
                                                 0) if event.key == 'escape' else None])
                if len(closed_set.keys()) % 10 == 0:
                    plt.pause(0.001)

            if current.x == goal_node.x and current.y == goal_node.y:
                print("Find goal")
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                break

            # Remove the item from the open set
            del open_set[c_id]

            # Add it to the closed set
            closed_set[c_id] = current

            # expand_grid search grid based on motion model
            for i, _ in enumerate(self.motion):
                node = self.Node(current.x + self.motion[i][0],
                                 current.y + self.motion[i][1],
                                 current.cost + self.motion[i][2], c_id)
                n_id = self.calc_grid_index(node)

                # If the node is not safe, do nothing
                if not self.verify_node(node):
                    continue

                if n_id in closed_set:
                    continue

                if n_id not in open_set:
                    open_set[n_id] = node  # discovered a new node
                else:
                    if open_set[n_id].cost > node.cost:
                        # This path is the best until now. record it
                        open_set[n_id] = node

        rx, ry = self.calc_final_path(goal_node, closed_set)

        return rx, ry, res

    def calc_final_path(self, goal_node, closed_set):
        # generate final course
        rx, ry = [self.calc_grid_position(goal_node.x, self.min_x)], [
            self.calc_grid_position(goal_node.y, self.min_y)]
        parent_index = goal_node.parent_index
        while parent_index != -1:
            n = closed_set[parent_index]
            rx.append(self.calc_grid_position(n.x, self.min_x))
            ry.append(self.calc_grid_position(n.y, self.min_y))
            parent_index = n.parent_index

        return rx, ry

    @staticmethod
    def calc_heuristic(n1, n2):
        w = 1.0  # weight of heuristic
        d = w * math.hypot(n1.x - n2.x, n1.y - n2.y)
        return d

    def calc_grid_position(self, index, min_position):
        """
        calc grid position

        :param index:
        :param min_position:
        :return:
        """
        pos = index * self.resolution + min_position
        return pos

    def calc_xy_index(self, position, min_pos):
        return round((position - min_pos) / self.resolution)

    def calc_grid_index(self, node):
        return (node.y - self.min_y) * self.x_width + (node.x - self.min_x)

    def verify_node(self, node):
        px = self.calc_grid_position(node.x, self.min_x)
        py = self.calc_grid_position(node.y, self.min_y)

        if px < self.min_x:
            return False
        elif py < self.min_y:
            return False
        elif px >= self.max_x:
            return False
        elif py >= self.max_y:
            return False

        # collision check
        if self.obstacle_map[node.x][node.y]:
            return False

        return True

    def calc_obstacle_map(self, ox, oy):

        self.min_x = round(min(ox))
        self.min_y = round(min(oy))
        self.max_x = round(max(ox))
        self.max_y = round(max(oy))

        self.x_width = round((self.max_x - self.min_x) / self.resolution)
        self.y_width = round((self.max_y - self.min_y) / self.resolution)
        # print("x_width:", self.x_width)
        # print("y_width:", self.y_width)

        # obstacle map generation
        self.obstacle_map = [[False for _ in range(self.y_width)]
                             for _ in range(self.x_width)]
        for ix in range(self.x_width):
            x = self.calc_grid_position(ix, self.min_x)
            for iy in range(self.y_width):
                y = self.calc_grid_position(iy, self.min_y)
                for iox, ioy in zip(ox, oy):
                    d = math.hypot(iox - x, ioy - y)
                    if d <= self.rr:
                        self.obstacle_map[ix][iy] = True
                        break

    @staticmethod
    def get_motion_model():
        # dx, dy, cost
        motion = [[1, 0, 1],
                  [0, 1, 1],
                  [-1, 0, 1],
                  [0, -1, 1],
                  [-1, -1, math.sqrt(2)],
                  [-1, 1, math.sqrt(2)],
                  [1, -1, math.sqrt(2)],
                  [1, 1, math.sqrt(2)]]

        return motion

class RandomTC:
    def __init__(self, top_vertex : list, bottom_vertex : list) -> None:
        self.ox = []
        self.oy = []

        self.obx = []
        self.oby = []
        self.top_vertex = top_vertex
        self.bottom_vertex = bottom_vertex

##### map weighted set ##############################################################
    def random_map(self, obstacle_number):
        # generate random map within given boundary
        self.ox.clear(), self.oy.clear()
        # generate boundary
        for i in range(self.bottom_vertex[0], self.top_vertex[0]):
            self.ox.append(i)
            self.oy.append(self.bottom_vertex[1])
        for i in range(self.bottom_vertex[1], self.top_vertex[1]):
            self.ox.append(self.top_vertex[0])
            self.oy.append(i)
        for i in range(self.bottom_vertex[0], self.top_vertex[0]+1):
            self.ox.append(i)
            self.oy.append(self.top_vertex[1])
        for i in range(self.bottom_vertex[1], self.top_vertex[1]+1):
            self.ox.append(self.bottom_vertex[0])
            self.oy.append(i)

        #generate obstacles
        for idx in range(obstacle_number):
            x_range = [np.random.randint(self.bottom_vertex[0] + 1, self.top_vertex[0]),
                       np.random.randint(self.bottom_vertex[0] + 1, self.top_vertex[0])]
            y_range = [np.random.randint(self.bottom_vertex[1] + 1, self.top_vertex[1]),
                       np.random.randint(self.bottom_vertex[1] + 1, self.top_vertex[1])]
            if x_range[0] > x_range[1]:
                x_range[0], x_range[1] = x_range[1], x_range[0]
            if y_range[0] > y_range[1]:
                y_range[0], y_range[1] = y_range[1], y_range[0]

            for i in range(x_range[0], x_range[1]):
                self.obx.append(i)
                self.oby.append(y_range[0])
            for i in range(y_range[0], y_range[1]):
                self.obx.append(x_range[1])
                self.oby.append(i)
            for i in range(x_range[0], x_range[1]+1):
                self.obx.append(i)
                self.oby.append(y_range[1])
            for i in range(y_range[0], y_range[1]+1):
                self.obx.append(x_range[0])
                self.oby.append(i)
        
        self.ox = self.ox + self.obx
        self.oy = self.oy + self.oby

        return self.ox, self.oy

    def random_coordinate(self):
        # generate random coordinates inside maze
        flag = True
        while flag:
            tempx = np.random.randint(self.bottom_vertex[0] + 1, self.top_vertex[0])
            tempy = np.random.randint(self.bottom_vertex[1] + 1, self.top_vertex[1])
            if not (tempx in self.obx and tempy in self.oby):
                break 
        coordinate = [tempx, tempy]
        return coordinate
    
#############################################################################################
##### destination weighted set ##############################################################
    def random_coordinate_d(self):
        # generate random coordinates inside maze
        self.coordinate = [np.random.randint(self.bottom_vertex[0] + 1, self.top_vertex[0]), np.random.randint(self.bottom_vertex[1] + 1, self.top_vertex[1])]
        return self.coordinate
    
    def random_map_d(self, obstacle_number):
        # generate random map within given boundary
        self.ox.clear(), self.oy.clear()
        # generate boundary
        for i in range(self.bottom_vertex[0], self.top_vertex[0]):
            self.ox.append(i)
            self.oy.append(self.bottom_vertex[1])
        for i in range(self.bottom_vertex[1], self.top_vertex[1]):
            self.ox.append(self.top_vertex[0])
            self.oy.append(i)
        for i in range(self.bottom_vertex[0], self.top_vertex[0]+1):
            self.ox.append(i)
            self.oy.append(self.top_vertex[1])
        for i in range(self.bottom_vertex[1], self.top_vertex[1]+1):
            self.ox.append(self.bottom_vertex[0])
            self.oy.append(i)

        #generate obstacles
        for idx in range(obstacle_number):
            x_range = [np.random.randint(self.bottom_vertex[0] + 1, self.top_vertex[0]),
                       np.random.randint(self.bottom_vertex[0] + 1, self.top_vertex[0])]
            y_range = [np.random.randint(self.bottom_vertex[1] + 1, self.top_vertex[1]),
                       np.random.randint(self.bottom_vertex[1] + 1, self.top_vertex[1])]
            if x_range[0] > x_range[1]:
                x_range[0], x_range[1] = x_range[1], x_range[0]
            if y_range[0] > y_range[1]:
                y_range[0], y_range[1] = y_range[1], y_range[0]

            for i in range(x_range[0], x_range[1]):
                self.obx.append(i)
                self.oby.append(y_range[0])
            for i in range(y_range[0], y_range[1]):
                self.obx.append(x_range[1])
                self.oby.append(i)
            for i in range(x_range[0], x_range[1]+1):
                self.obx.append(i)
                self.oby.append(y_range[1])
            for i in range(y_range[0], y_range[1]+1):
                self.obx.append(x_range[0])
                self.oby.append(i)
        
        self.ox = self.ox + self.obx
        self.oy = self.oy + self.oby

        return self.ox, self.oy
#############################################################################################

# Params : top_vertex : list = [60,60], bottom_vertex : list = [0,0], resolution  : float = 2.0, robot_radius : float= 1.0, obstacle_num : int = 5
def astar_planner_go(yaml_path_pln):
    with open(yaml_path_pln) as f:
        print("reading planner config: " + yaml_path_pln + "...")
        planner_config = yaml.load(f, Loader=yaml.FullLoader)

    # map generator
    map_gen = RandomTC(planner_config['parameters']['top_vertex'], planner_config['parameters']['bottom_vertex'])
    ox, oy = map_gen.random_map(planner_config['parameters']['obstacle_num'])
    start = map_gen.random_coordinate()
    end = map_gen.random_coordinate()

    if show_animation:  # pragma: no cover
        plt.plot(ox, oy, ".k")
        plt.plot(start[0], start[1], "og")
        plt.plot(end[0], end[1], "xb")
        plt.grid(True)
        plt.axis("equal")
    
    a_star = AStarPlanner(ox, oy, planner_config['parameters']['grid_size'], planner_config['parameters']['robot_radius'])
    rx, ry, res = a_star.planning(start[0], start[1], end[0], end[1]) # type : list, list

    # show path
    if show_animation:  # pragma: no cover
        plt.plot(rx, ry, "-r")
        plt.pause(0.001)
        plt.show(block=False)
        plt.pause(3)
        print("closing")
        plt.close('all')

    return rx, ry, res
    

# global_pp_yaml
def main(global_pp_yaml):
    astar_planner_go(os.path.join(os.environ["GOLE_CORE_CONFIG"], global_pp_yaml))
   


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Teleoperation script for GoLeRobotics Platforms")
    parser.add_argument("-k", "--global_pp_yaml", type=str, default=None, help="Global Planner YAML Config file path.")
    args = parser.parse_args()
    main(args.global_pp_yaml)