import sys
import random
import heapq
import math

n = 200
robot_num = 10
berth_num = 10
N = 210


def obs_map(cmap):
    """
    坐标转换
    :return: 返回地图障碍坐标
    """
    obs = set()
    for x in range(200):
        for y in range(200):
            if cmap[y][x] in ['*', '#']:  # y为向右正向，x为向下正向
                obs.add([x, y])  # 是障碍物则记录坐标
    return obs


class AStar:
    """AStar set the cost + heuristics as the priority
    """
    def __init__(self, s_start, s_goal, heuristic_type, obs_position):
        self.s_start = s_start
        self.s_goal = s_goal
        self.heuristic_type = heuristic_type

        self.u_set = [(-1, 0), (0, 1), (1, 0), (0, -1)]  # feasible input set
        self.obs = obs_position  # position of obstacles

        self.OPEN = []  # priority queue / OPEN set
        self.CLOSED = []  # CLOSED set / VISITED order
        self.PARENT = dict()  # recorded parent
        self.g = dict()  # cost to come

    def searching(self):
        """
        A*算法
        :return: 路径和遍历点
        """

        self.PARENT[self.s_start] = self.s_start
        self.g[self.s_start] = 0
        self.g[self.s_goal] = math.inf
        heapq.heappush(self.OPEN,
                       (self.f_value(self.s_start), self.s_start))

        while self.OPEN:
            _, s = heapq.heappop(self.OPEN)
            self.CLOSED.append(s)

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

        return self.extract_path(self.PARENT), self.CLOSED


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

    def action(self, robot_id, cargo_gds, berth_gds):
        """
        移动，先移动后进行取货放货
        :param robot_id: 机器人id
        :param cargo_gds: 货物的位置列表
        :param berth_gds: 泊位的位置列表
        :return: None
        """
        # 状态为0停止运行（随机走）
        if self.status == 0:
            print("move", robot_id, random.randint(0, 3))

        # 正常运行
        else:
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
                # 向最近的货物位置移动




                # 检查将移动的位置是否有效
                new_pos = None
                print("move", robot_id, random.randint(0, 3))
                # 判断是否到了最优点，到了取货就行取货物
                if new_pos == nearest_cargo:
                    print("get", robot_id)
            else:  # 有货物，因此找泊位


                # 检查将移动的位置是否有效
                print("move", robot_id, random.randint(0, 3))


robot = [Robot() for _ in range(robot_num + 10)]


class Berth:
    def __init__(self, x=0, y=0, transport_time=0, loading_speed=0):
        self.x = x
        self.y = y
        self.transport_time = transport_time
        self.loading_speed = loading_speed


berth = [Berth() for _ in range(berth_num + 10)]


class Boat:
    def __init__(self, num=0, pos=0, status=0):
        self.num = num
        self.pos = pos
        self.status = status


boat = [Boat() for _ in range(10)]

money = 0
boat_capacity = 0
id = 0
ch = []
gds = [[0 for _ in range(N)] for _ in range(N)]
axis = []


def Init():
    for i in range(0, n):
        line = input()
        ch.append([c for c in line.split(sep=" ")])
    for i in range(berth_num):
        line = input()
        berth_list = [int(c) for c in line.split(sep=" ")]
        id = berth_list[0]
        berth[id].x = berth_list[1]
        berth[id].y = berth_list[2]
        berth[id].transport_time = berth_list[3]
        berth[id].loading_speed = berth_list[4]
    boat_capacity = int(input())
    okk = input()
    print("OK")
    sys.stdout.flush()


def Input():
    id, money = map(int, input().split(" "))
    num = int(input())
    for i in range(num):
        x, y, val = map(int, input().split())
        gds[x][y] = val
        axis.append((x, y))
    for i in range(robot_num):
        robot[i].goods, robot[i].x, robot[i].y, robot[i].status = map(int, input().split())
    for i in range(5):
        boat[i].status, boat[i].pos = map(int, input().split())
    okk = input()
    return id


if __name__ == "__main__":
    Init()
    obs_position = obs_map(ch)
    for zhen in range(1, 15001):
        id = Input()
        for i in range(robot_num):
            print("move", i, random.randint(0, 3))
            sys.stdout.flush()
        print("OK")
        sys.stdout.flush()
