import math, json, time,sys, tracemalloc
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

show_animation = True
road = []
counter = 999


class UniformCostSearch:
    def __init__(self, start, goal, grid, grid_size, radius, dimension, motion):
        self.grid = grid
        self.grid_size = grid_size
        self.radius = radius
        self.sx = start[0]
        self.sy = start[1]
        self.gx = goal[0]
        self.gy = goal[1]
        self.start_node = [self.sx, self.sy, 0.0, -1, []] # x, y, cost, index, parent_node
        self.goal_node = [self.gx, self.gy, 0.0, -1, []] # x, y, cost, index, parent_node
        self.ox, self.oy = [], []
        self.fx, self.fy = [], []
        self.open_set, self.closed_set = dict(), dict()
        self.motion = motion
        
        self.calc_obstacle_map()

        if show_animation:  
            plt.figure(figsize=(dimension ,dimension))
            plt.xlim(-1, grid.shape[0])
            plt.ylim(-1, grid.shape[1])
            plt.xticks(np.arange(0,grid.shape[0],1))
            plt.yticks(np.arange(0,grid.shape[1],1))
            plt.grid(True)
            plt.scatter(self.sx, self.sy, s=400, c='red', marker='s')
            plt.scatter(self.gx, self.gy, s=400, c='green', marker='s')
            plt.scatter(self.ox, self.oy, s=300, c='gray', marker='s')
            plt.scatter(self.fx, self.fy, s=300, c='cyan', marker='s')  

        self.open_set[self.calc_grid_index(self.start_node)] = self.start_node

    def get_possible_moves(self):
        self.c_id = min(self.open_set, key=lambda o: self.open_set[o][2])
        self.current = self.open_set[self.c_id]

        del self.open_set[self.c_id] # previously is pop, now is select specific index to delete
        self.closed_set[self.c_id] = self.current
        
        if show_animation:
                plt.scatter(self.current[0], self.current[1], s=300, c='yellow', marker='s')    

                plt.gcf().canvas.mpl_connect('key_release_event',
                                             lambda event:
                                             [exit(0) if event.key == 'escape'
                                              else None])

        if len(self.closed_set) >= 2:
                locx, locy = self.cal_path_distance(self.current)
                for i in range(len(locx)-1):
                    px = (locx[i], locx[i+1])
                    py = (locy[i], locy[i+1])
                    plt.plot(px, py, "-m", linewidth=4)

        return True

    def goal_found(self):
        if self.current[0] == self.gx and self.current[1] == self.gy:
            print("Find goal")

            plt.scatter(self.current[0],
                        self.current[1],
                        s=300, c='yellow', marker='s') 
            
            if len(self.closed_set) >= 2:
                locx, locy = self.cal_path_distance(self.current)
                for i in range(len(locx)-1):
                    px = (locx[i], locx[i+1])
                    py = (locy[i], locy[i+1])
                    plt.plot(px, py, "-m", linewidth=4)

            self.goal_node[3] = self.current[3]
            self.goal_node[2] = self.current[2]
            
            print("cost:", self.goal_node[2] )
            self.rx, self.ry = self.cal_path_distance(self.goal_node)
            return True
        return False

    def explore_next_move(self):
        potential_moves = self.generate_potential_moves()
        
        for move in potential_moves:
            node = [self.current[0] + move[0], self.current[1] + move[1], self.current[2] + move[2], self.c_id, []]
            n_id = self.calc_grid_index(node)

            # If the node is not safe, do nothing
            if not self.valid_move(node):
                plt.scatter(node[0],
                            node[1],
                            s=100, c='black', marker='s')
                continue

            if (n_id not in self.open_set):
                node[4] = self.current
                self.open_set[n_id] = node
                plt.scatter(node[0],
                            node[1],
                            s=100, c='blue', marker='s')
            else:
                if self.open_set[n_id][2] > node[2]:
                    self.open_set[n_id] = node

                    plt.scatter(node[0],
                                node[1],
                                s=300, c='green', marker='s')

    # Helper Functions

    def generate_potential_moves(self):
        if(self.motion == '4n'):
            motion = [[1, 0, 1],
                    [0, 1, 1],
                    [-1, 0, 1],
                    [0, -1, 1]]
        else:
            motion = [[1, 0, 1],
                  [1, 1, math.sqrt(2)],
                  [0, 1, 1],
                  [-1, 1, math.sqrt(2)],
                  [-1, 0, 1],
                  [-1, -1, math.sqrt(2)],
                  [0, -1, 1],
                  [1, -1, math.sqrt(2)]]
        return motion

    def valid_move(self, node):
        px = node[0]
        py = node[1]

        if px < self.min_x:
            return False
        elif py < self.min_y:
            return False
        elif px >= self.maxx:
            return False
        elif py >= self.maxy:
            return False

        # collision check
        if self.obmap[node[0]][node[1]]:
            return False

        return True

    def calc_grid_index(self, node):
        self.xwidth = round((round(max(self.ox)) - round(min(self.ox))) / 1)
        self.ywidth = round((round(max(self.oy)) - round(min(self.oy))) / 1)
        return (node[1] - round(min(self.oy))) * self.xwidth + (node[0] - round(min(self.ox)))

    def calc_obstacle_map(self):
        for row in range(self.grid.shape[0]):
            for column in range(self.grid.shape[1]):
                if self.grid[row, column] == 1:
                    self.ox.append(column)
                    self.oy.append(row)
                else:
                    self.fx.append(column)
                    self.fy.append(row)

        self.min_x = round(min(self.ox))
        self.min_y = round(min(self.oy))
        self.maxx = round(max(self.ox))
        self.maxy = round(max(self.oy))

        self.xwidth = round((self.maxx - self.min_x) / self.grid_size)
        self.ywidth = round((self.maxy - self.min_y) / self.grid_size)


        # obstacle map generation
        self.obmap = [[False for _ in range(self.ywidth)]
                      for _ in range(self.xwidth)]
        for ix in range(self.xwidth):
            x = ix
            for iy in range(self.ywidth):
                y = iy
                for iox, ioy in zip(self.ox, self.oy):
                    d = math.hypot(iox - x, ioy - y)
                    if d < self.radius:
                        self.obmap[ix][iy] = True
                        break

    def cal_path_distance(self, current):
        dx, dy = [current[0]], [current[1]]
        index = current[3]
        c = 0
        while index != -1:
            node = self.closed_set[index]
            dx.append(node[0])
            dy.append(node[1])
            index = node[3]
            c = c + 1
            if c == counter:
                print('There is an infinite loop')
                sys.exit()
        return dx, dy
        
# Init
motion_option = input("Enter the motion (4n/8n): ")
if(motion_option != '4n') and (motion_option != '8n'):
    print("Bye, please try with another motion")
    sys.exit()

config_file = './Groupassessment/config9x9.json'
with open(config_file) as config_env:
        param = json.load(config_env)

start = param['start']
goal = param['goal']
grid_size = param['resolution']
robot_radius = param['robot_radius']
map_xlsx = param['map_xlsx']
fig_dim = param['fig_dim']
gmap = pd.read_excel(map_xlsx,header=None)
grid = gmap.to_numpy()
grid = grid[::-1]

tracemalloc.start()
start_time = time.time()
bfs = UniformCostSearch(start, goal, grid, grid_size, robot_radius, fig_dim, motion_option)
while True:
    # Determine next possible moves.
    bfs.get_possible_moves()
    if bfs.goal_found():
        break
    bfs.explore_next_move()
end_time = time.time()
current, peak = tracemalloc.get_traced_memory()
tracemalloc.stop()

bfs.rx.reverse()  
bfs.ry.reverse()
new_rx = map(int, bfs.rx)
new_ry = map(int, bfs.ry)
road = [list(a) for a in zip(new_rx, new_ry)]
print("road from start to goal", road)

print(f"Current memory usage is {current / 10**6}MB; Peak was {peak / 10**6}MB")
print("time end to calculate time")
print("time used", end_time - start_time)

if show_animation:  
    for i in range(len(bfs.rx)-1):
        px = (bfs.rx[i], bfs.rx[i+1])
        py = (bfs.ry[i], bfs.ry[i+1])
        plt.plot(px, py, "-k")
        plt.pause(0.1)
    plt.show() 