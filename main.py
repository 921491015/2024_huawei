import sys
import random
import heapq
import math

n = 200
robot_num = 10
berth_num = 10
N = 210

def berth_expand(axis):
    x, y = axis
    berth_expand_set = []
    # 向右扩张三格
    for dx in range(0, 4):
        new_x = x + dx
        for dy in range(0, 4):  # 向下扩张三格
            new_y = y + dy
            # 将新的坐标添加到结果列表中
            berth_expand_set.append((new_x, new_y))

    return berth_expand_set


def obs_map(cmap):
    """
    坐标转换
    :return: 返回地图障碍坐标
    """
    ch_array = [[char for char in sublist[0]] for sublist in cmap]
    obs = set()
    for x in range(200):
        for y in range(200):
            if ch_array[y][x] in ['*', '#']:  # y为向右正向，x为向下正向
                obs.add((y, x))  # 是障碍物则记录坐标
    return obs


def move_direction(current_pos, next_pos):
    dx, dy = next_pos[0] - current_pos[0], next_pos[1] - current_pos[1]
    if dx == 1:
        return 3  # 下移
    elif dx == -1:
        return 2  # 上移
    elif dy == 1:
        return 0  # 右移
    elif dy == -1:
        return 1  # 左移
    else:
        return random.randint(0, 3)


class AStar:
    """AStar set the cost + heuristics as the priority
    """
    def __init__(self, s_start, s_goal, obs_position):
        self.s_start = s_start
        self.s_goal = s_goal

        self.u_set = [(-1, 0), (0, 1), (1, 0), (0, -1)]  # feasible input set
        self.obs = obs_position  # position of obstacles

        self.OPEN = []  # priority queue / OPEN set
        self.CLOSED = []  # CLOSED set / VISITED order
        self.PARENT = dict()  # recorded parent
        self.g = dict()  # cost to come

    def searching(self, max_iterations=40000):
        """
        A*算法
        :return: 路径和遍历点
        """
        iteration = 0
        self.PARENT[self.s_start] = self.s_start
        self.g[self.s_start] = 0
        self.g[self.s_goal] = math.inf
        heapq.heappush(self.OPEN,
                       (self.f_value(self.s_start), self.s_start))

        while self.OPEN and iteration < max_iterations:
            iteration += 1
            _, s = heapq.heappop(self.OPEN)
            # self.CLOSED.append(s)

            if s == self.s_goal:  # 停止的情况
                break

            for s_n in self.get_neighbor(s):
                new_cost = self.g[s] + self.cost(s, s_n)

                if s_n not in self.g:
                    self.g[s_n] = math.inf

                if new_cost < self.g[s_n]:  # 更新代价情况
                    self.g[s_n] = new_cost
                    self.PARENT[s_n] = s
                    heapq.heappush(self.OPEN, (self.f_value(s_n), s_n))
        if iteration >= max_iterations:
            return None

        return self.extract_path(self.PARENT)


    def get_neighbor(self, s):
        """
        找到不是障碍的领点
        :param s: 状态
        :return: 邻点
        """

        return [(s[0] + u[0], s[1] + u[1]) for u in self.u_set]

    def cost(self, s_start, s_goal):
        """
        计算此次移动的代价
        :param s_start: 起始节点
        :param s_goal: 结束节点
        :return:  移动代价
        :note: 这个计算比较复杂
        """

        if self.is_collision(s_start, s_goal):
            return math.inf

        return math.hypot(s_goal[0] - s_start[0], s_goal[1] - s_start[1])

    def is_collision(self, s_start, s_end):
        """
        判断是否碰边
        :param s_start: 起始节点
        :param s_end: 终止节点
        :return: True: 碰 / False: 没碰
        """

        if s_start in self.obs or s_end in self.obs:
            return True

        if s_start[0] != s_end[0] and s_start[1] != s_end[1]:
            if s_end[0] - s_start[0] == s_start[1] - s_end[1]:
                s1 = (min(s_start[0], s_end[0]), min(s_start[1], s_end[1]))
                s2 = (max(s_start[0], s_end[0]), max(s_start[1], s_end[1]))
            else:
                s1 = (min(s_start[0], s_end[0]), max(s_start[1], s_end[1]))
                s2 = (max(s_start[0], s_end[0]), min(s_start[1], s_end[1]))

            if s1 in self.obs or s2 in self.obs:
                return True

        return False

    def f_value(self, s):
        """
        启发式函数
        :param s: 当前状态
        :return: f
        """

        return self.g[s] + self.heuristic(s)

    def extract_path(self, PARENT):
        """
        根据父亲节点提取路径
        :return: 规划路径
        """

        path = [self.s_goal]
        s = self.s_goal

        while True:
            s = PARENT[s]
            path.append(s)

            if s == self.s_start:
                break

        return list(path)

    def heuristic(self, s):
        """
        曼哈顿距离
        :param s: 当前节点
        :return: h代价
        """

        goal = self.s_goal  # goal node
        return abs(goal[0] - s[0]) + abs(goal[1] - s[1])


class Robot:
    def __init__(self, startX=0, startY=0, goods=0, status=0, mbx=0, mby=0):
        self.x = startX
        self.y = startY
        self.goods = goods  # 0:空 1:满
        self.status = status  # 0:待机故障 1:正常运行
        self.mbx = mbx
        self.mby = mby
        self.crash_flag = False  # 是否发生故障标志，为了重启
        self.wait_total_num = random.randint(0, 3)  # 等待几回合,随机的,每个机器人不一样
        self.wait_num = 0  # 记录等待的回合
        self.minimum_dis = float('inf')  # 记录当前最小货物距离，是全局性的缓存
        self.min_cargo_pos = (1, 2)  # 记录当前最近货物位置
        self.good_get_fix_flag = 0
        self.path_length = 0


    def action(self, robot_id, cargo_gds, berth_gds):
        """
        移动，先移动后进行取货放货
        :param robot_id: 机器人id
        :param cargo_gds: 货物的位置列表
        :param berth_gds: 泊位的位置列表
        :return: None
        """
        global axis
        global all_robot_path_go_berth
        global all_robot_path_find_cargo
        global path_has_find_flag_for_berth
        global path_has_find_flag_for_cargo
        sys.stderr.write('robot_id:' + str(robot_id)+'\n')
        # sys.stderr.write('goods:' + str(self.goods)+'\n')
        # sys.stderr.write('STATUS:' + str(self.status)+'\n')
        # sys.stderr.write('cargo_num:' + str(len(cargo_gds))+'\n')
        # 状态为0停止运行（随机走）
        if self.status == 0:
            # print("move", robot_id, random.randint(0, 3))
            # sys.stdout.flush()
            self.crash_flag = True
            path_has_find_flag_for_berth[robot_id] = False
            # path_has_find_flag_for_cargo[robot_id] = False
            pass

        # 正常运行
        elif self.status == 1:
            if self.crash_flag:  # 机器人前面发生了碰撞
                if self.wait_num < self.wait_total_num:  # 机器人的重启时长
                    self.wait_num += 1  # 累加
                else:
                    self.wait_num = 0  # 累计数清零
                    self.crash_flag = False  # 标志位清零
                    print("move", robot_id, random.randint(0, 3))  # 并随机走，为了错开
            else:  # 前面机器人没有发生碰撞
                if self.goods == 0:  # 没有货物，因此找货物
                    current_pos = (self.x, self.y)
                    # 寻找最近的货物，每次进入循环计算，是贪心
                    nearest_cargo = None
                    min_distance = float('inf')
                    for cargo_pos in cargo_gds:
                        distance = abs(cargo_pos[0] - current_pos[0]) + abs(cargo_pos[1] - current_pos[1])
                        if distance < min_distance:
                            min_distance = distance
                            nearest_cargo = cargo_pos
                    if min_distance + self.path_length < self.minimum_dis:
                        self.minimum_dis = min_distance
                        self.min_cargo_pos = nearest_cargo
                        # 向最近的货物位置移动
                        sys.stderr.write("miaomiaomiaomiaomiaomiaomiaomiaomiaomiaomiaomiao\n")
                        astar = AStar(current_pos, nearest_cargo, obs_position)
                        path = astar.searching()
                        if not path:
                            axis.remove(nearest_cargo)
                            self.minimum_dis = float('inf')
                        else:
                            all_robot_path_find_cargo[robot_id] = path
                            new_pos = all_robot_path_find_cargo[robot_id][-2]
                            all_robot_path_find_cargo[robot_id].pop()
                            self.path_length += 1
                            dr = move_direction(current_pos, new_pos)
                            print("move", robot_id, dr)
                            sys.stdout.flush()
                            # 判断是否到了最优点，到了取货就行取货物
                            if new_pos == nearest_cargo:
                                self.minimum_dis = float('inf')
                                self.path_length = 0
                                # if self.good_get_fix_flag == 0:
                                #     self.good_get_fix_flag = 1
                                # elif self.good_get_fix_flag == 1:
                                #     self.good_get_fix_flag = 0
                                print("get", robot_id)
                                sys.stdout.flush()
                                axis.remove(nearest_cargo)
                    else:
                        sys.stderr.write("HAHAHAHHAHAHAHAHA\n")
                        new_pos = all_robot_path_find_cargo[robot_id][-2]
                        all_robot_path_find_cargo[robot_id].pop()
                        self.path_length += 1
                        dr = move_direction(current_pos, new_pos)
                        print("move", robot_id, dr)
                        sys.stdout.flush()
                        # 判断是否到了最优点，到了取货就行取货物
                        if new_pos == self.min_cargo_pos:
                            self.minimum_dis = float('inf')
                            self.path_length = 0
                            print("get", robot_id)
                            sys.stdout.flush()
                            axis.remove(self.min_cargo_pos)


                elif self.goods == 1:  # 有货物，因此找泊位
                    current_pos = (self.x, self.y)
                    # 直接对应去送对应的港口，不管了
                    target_berth = berth_gds[robot_id]
                    if not path_has_find_flag_for_berth[robot_id]:
                        # 向最近的货物位置移动
                        astar = AStar(current_pos, target_berth, obs_position)
                        path = astar.searching()
                        path_has_find_flag_for_berth[robot_id] = True
                        all_robot_path_go_berth[robot_id] = path
                        new_pos = all_robot_path_go_berth[robot_id][-2]
                        all_robot_path_go_berth[robot_id].pop()
                    # 检查将移动的位置是否有效
                        dr = move_direction(current_pos, new_pos)
                        print("move", robot_id, dr)
                        sys.stdout.flush()
                        # 判断是否到了最优点，到了放货
                        if new_pos == target_berth:
                            path_has_find_flag_for_berth[robot_id] = False
                            print("pull", robot_id)
                            sys.stdout.flush()

                    else:
                        sys.stderr.write("GAGAGAGAGAGAGAGAGAGA\n")
                        new_pos = all_robot_path_go_berth[robot_id][-2]
                        all_robot_path_go_berth[robot_id].pop()
                        dr = move_direction(current_pos, new_pos)
                        print("move", robot_id, dr)
                        sys.stdout.flush()
                        if new_pos == target_berth:
                            path_has_find_flag_for_berth[robot_id] = False
                            print("pull", robot_id)
                            sys.stdout.flush()


robot = [Robot() for _ in range(robot_num + 10)]


class Berth:
    def __init__(self, x=0, y=0, transport_time=0, loading_speed=0, cargo_num=0):
        self.x = x
        self.y = y
        self.transport_time = transport_time
        self.loading_speed = loading_speed
        self.cargo_save = 0
        self.cargo_save_now = 0
        self.berth_id = 0


berth = [Berth() for _ in range(berth_num + 10)]


class Boat:
    def __init__(self, num=0, pos=0, status=0):
        self.num = num
        self.pos = pos  # 0-9是目标泊位 -1虚拟点
        self.status = status  # 0：运输中，1：正常运行状态，运输完成或者装卸货，2：泊位外等待
        self.boat_goal_flag = 0
        self.wait_time = 0 #已经等待的时间
        self.last_time = 0  # 最后一轮规定等待的时间
        self.first = 1 #第一次派船
        self.last  = 0
        self.go = 0 #返回港口
        self.berth_repon = ()
    def action_boat_random(self, boat_id):
        """
        船的动作规定，一个船负责两个停泊口，起码前期就先这样工作，目前没有思考全局优化，两个泊位
        进行局部优化比较好。
        :return: None
        """
        global boat_load_time
        if self.status == 0:  # 在运输
            pass
        else:  # 在运行
            if self.pos == -1:  # 在虚拟点，需要移动到泊位上，这个地方直接就是直接交替到的，没有算法，没有优化，先测试
                boat_load_time[boat_id] = random.randint(100, 500)  # 在船去之前给船一个装载时间，这个全部是测试阶段的，没有计算货物数量，没有优化
                if self.boat_goal_flag:  # 移动到A位
                    print("ship", boat_id, boat_id * 2)
                    sys.stdout.flush()
                    self.boat_goal_flag = False
                else:  # 移动到B位
                    print("ship", boat_id, boat_id * 2 + 1)
                    sys.stdout.flush()
                    self.boat_goal_flag = True
            else:  # 未移动到虚拟点,还装装货
                if self.wait_time < boat_load_time[boat_id]:
                    self.wait_time += 1
                    pass
                else:
                    self.wait_time = 0
                    print("go", boat_id)
                    sys.stdout.flush()

    def action_boat(self, boat_id):
        """
        船的动作规定，一个船负责两个停泊口，起码前期就先这样工作，目前没有思考全局优化，两个泊位
        进行局部优化比较好。
        :return: None
        """
        global boat_load_time
        if 15000 - 2* (berth[self.berth_repon[0]].transport_time - berth[self.berth_repon[1]].transport_time) - 500 <= id and self.last == 0:
            if self.go == 1 and self.status == 0:
                #编号为self.berth_repon[self.boat_goal_flag]机器人往港口编号为self.berth_repon[int(not self.boat_goal_flag)]的地方送货
                self.last = 1
            elif self.go == 0 and self.status == 0:
                # 编号为self.berth_repon[int(not self.boat_goal_flag)]机器人往港口编号为self.berth_repon[self.boat_goal_flag]的地方送货
                self.last = 0.5
            elif self.pos == -1 and self.status == 1:
                # 编号为self.berth_repon[self.boat_goal_flag]机器人往港口编号为self.berth_repon[int(not self.boat_goal_flag)]的地方送货
                self.last = 0.5
            else:
                # 编号为self.berth_repon[int(not self.boat_goal_flag)]机器人往港口编号为self.berth_repon[self.boat_goal_flag]的地方送货
                self.last = 0.5



        if self.status == 0:  # 在运输
            pass
        else:  # 在运行
            if self.pos == -1:  # 在虚拟点，需要移动到泊位上，这个地方直接就是直接交替到的，没有算法，没有优化，先测试
                if self.first == 1:   #第一次派船
                    max_index = max((0, 1), key=lambda index: berth[self.berth_repon[index]].loading_speed)
                    sys.stderr.write('第一次派船' + str(self.berth_repon[max_index]) + '\n')
                    sys.stderr.write('移动的时间' + str(berth[self.berth_repon[max_index]].transport_time) + '\n')
                    # boat_load_time[boat_id] = random.randint(100, 500)
                    print("ship", boat_id, self.berth_repon[max_index])
                    sys.stdout.flush()
                    self.boat_goal_flag = max_index

                else:
                # boat_load_time[boat_id] = random.randint(100, 500)  # 在船去之前给船一个装载时间，这个全部是测试阶段的，没有计算货物数量，没有优化
                #     if self.wait_time
                    self.go = 0
                    if self.boat_goal_flag:  # 移动到A位
                        print("ship", boat_id, self.berth_repon[0])
                        sys.stdout.flush()
                        self.boat_goal_flag = 0
                    else:  # 移动到B位
                        print("ship", boat_id, self.berth_repon[1])
                        sys.stdout.flush()
                        self.boat_goal_flag = 1
            else:  # 未移动到虚拟点,还装装货
                # if self.arrive == 0:
                #     self.arrive = 1
                if self.first == 1:
                    self.first = 0
                if self.last == 0.5:
                    self.last_time = id - berth[self.berth_repon[self.boat_goal_flag]].transport_time - 2 * berth[
                        self.berth_repon[int(not self.boat_goal_flag)]].transport_time - berth[self.berth_repon[
                        int(not self.boat_goal_flag)]].cargo_save * berth[self.berth_repon[int(
                        not self.boat_goal_flag)]].loading_speed - 100
                    self.num = min(self.wait_time // berth[self.berth_repon[self.boat_goal_flag]].loading_speed, berth[self.berth_repon[self.boat_goal_flag]].cargo_save)
                    if self.wait_time >= self.last_time or boat_capacity <= self.num + 1:
                        self.wait_time = 0
                        berth[self.berth_repon[self.boat_goal_flag]].cargo_save = max(0, self.num - berth[self.berth_repon[self.boat_goal_flag]].cargo_save)
                        self.num = 0
                        self.last = 1
                        print("go", boat_id)
                        sys.stdout.flush()
                        self.go = 1

                    if self.go != 1:
                        self.wait_time += 1
                elif self.last == 1:
                    self.last_time = id - berth[self.berth_repon[self.boat_goal_flag]].transport_time - 100
                    if self.wait_time >= self.last_time:
                        self.wait_time = 0
                        berth[self.berth_repon[self.boat_goal_flag]].cargo_save = max(0, self.num - berth[self.berth_repon[self.boat_goal_flag]].cargo_save)
                        self.num = 0
                        self.last = 1
                        print("go", boat_id)
                        sys.stdout.flush()
                        self.go = 1
                    if self.go != 1:
                        self.wait_time += 1

                else:
                    self.num = min(self.wait_time // berth[self.berth_repon[self.boat_goal_flag]].loading_speed, berth[self.berth_repon[self.boat_goal_flag]].cargo_save)
                    sys.stderr.write('船在港口等待' + str(berth[self.berth_repon[self.boat_goal_flag]].berth_id) + '    船装的货物' + str(self.num) + '     港口的货物' + str(berth[self.berth_repon[self.boat_goal_flag]].cargo_save) + '\n')
                    # sys.stderr.write('船在港口装货的速度' + str(berth[int(not self.boat_goal_flag)].loading_speed)  + '    船等待的时间'  + str(self.wait_time) + '\n')
                    if boat_capacity <= self.num + 1  or (self.num > 9 and berth[self.berth_repon[self.boat_goal_flag]].cargo_save <= self.num) or (self.num < 9 and berth[self.berth_repon[int(not self.boat_goal_flag)]].cargo_save >= boat_capacity // 2):  #直接走
                        # if self.num + berth[self.boat_goal_flag].cargo_save
                        self.wait_time = 0
                        berth[self.berth_repon[self.boat_goal_flag]].cargo_save = max(0, self.num - berth[self.berth_repon[self.boat_goal_flag]].cargo_save)
                        self.num = 0

                        print("go", boat_id)
                        sys.stdout.flush()
                        self.go = 1
                    # berth[int(not self.boat_goal_flag)].cargo_save = 0
                # if self.wait_time < boat_load_time[boat_id]:
                #     if boat_capacity <= self.num and boat_capacity <= berth[self.berth_repon[self.boat_goal_flag]].cargo_save:
                #         self.wait_time = 0
                #         berth[self.berth_repon[self.boat_goal_flag]].cargo_save = 0
                #         self.num = 0
                #
                #         print("go", boat_id)
                #         sys.stdout.flush()
                #         self.go = 1
                        # berth[int(not self.boat_goal_flag)].cargo_save = 0
                        # if self.wait_time < boat_load_time[boat_id]:
                    if self.go != 1:
                        self.wait_time += 1

                #     pass
                # else:
                #     self.wait_time = 0
                #     print("go", boat_id)
                #     sys.stdout.flush()


boat = [Boat() for _ in range(10)]  # 船对象初始化

def get_berth_pair():
    # 根据实例的'value'属性进行排序
    sorted_berth = sorted(berth[:10], key=lambda instance: instance.transport_time)
    # 找到中间的索引
    mid = len(sorted_berth) // 2
    # 初始化结果列表
    pairs = []
    # 从排序后的列表两端取实例进行组合
    for i in range(mid):
        pair = (sorted_berth[i].berth_id, sorted_berth[-(i + 1)].berth_id)
        pairs.append(pair)
    sys.stderr.write('指派区域' + str(pairs) + '\n')
    return pairs

money = 0
boat_capacity = 0
id = 0
ch = []
gds = [[0 for _ in range(N)] for _ in range(N)]
axis = []  # 货物的坐标点集
cargo_dont_find = set()  # 货物没有找到的集合,留下一个接口后面处理
obs_position = set()  # 障碍点的初始化
berth_axis = []  # 泊位的坐标点集
robot_current_axis_save = [(i, 1) for i in range(0, 10)]
robot_past_axis_save = [(i, 1) for i in range(0, 10)]
boat_load_time = [i for i in range(5)]
berth_pair = [0 for _ in range(5)]
robot_path = []
all_robot_path_find_cargo = [robot_path.copy() for _ in range(10)]  # 所有机器人的路径,存在全局里
all_robot_path_go_berth = [robot_path.copy() for _ in range(10)]  # 所有机器人的路径,存在全局里
# path_has_find_flag_for_cargo = [False for _ in range(10)]  # 所有机器人找到路径的标志位，默认False，则第一次进入会找路径，找到路径后会置为True
path_has_find_flag_for_berth = [False for _ in range(10)]  # 所有机器人找到路径的标志位，默认False，则第一次进入会找路径，找到路径后会置为True

def Init():
    for i in range(0, n):
        line = input()
        ch.append([c for c in line.split(sep=" ")])
    for i in range(berth_num):
        line = input()
        berth_list = [int(c) for c in line.split(sep=" ")]
        id = berth_list[0]
        berth[id].berth_id = id
        berth[id].x = berth_list[1]
        berth[id].y = berth_list[2]
        berth[id].transport_time = berth_list[3]
        berth[id].loading_speed = berth_list[4]
    global boat_capacity
    boat_capacity = int(input())
    okk = input()
    global obs_position
    obs_position = obs_map(ch)
    global berth_axis
    berth_axis = [(berth[i].x, berth[i].y) for i in range(10)]  # 泊位坐标，还没优化，因为只计算左上角的点
    global berth_pair
    berth_pair = get_berth_pair()
    for i in range(5):
        boat[i].berth_repon = berth_pair[i]
        sys.stderr.write('船对应港口初始化' + str(boat[i].berth_repon) + '\n')
    print("OK")
    sys.stdout.flush()


def Input():
    id, money = map(int, input().split(" "))
    num = int(input())
    for i in range(num):
        x, y, val = map(int, input().split())
        gds[x][y] = val
        global axis
        axis.append((x, y))
    for i in range(robot_num):
        robot[i].goods, robot[i].x, robot[i].y, robot[i].status = map(int, input().split())
        global robot_current_axis_save
        robot_current_axis_save[i] = (robot[i].x, robot[i].y)
    for i in range(5):
        boat[i].status, boat[i].pos = map(int, input().split())
    okk = input()
    return id


if __name__ == "__main__":
    Init()
    for zhen in range(1, 15001):
        # robot_past_axis_save = robot_current_axis_save
        id = Input()
        for i in range(robot_num):
            try:
                robot[i].action(i, axis, berth_axis)
            except:
                pass
        for j in range(5):
            try:
                boat[j].action_boat(j)
            except:
                pass
        print("OK")
        sys.stdout.flush()