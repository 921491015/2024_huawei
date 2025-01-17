import copy
import sys
import random
import heapq
import math

n = 200
robot_num = 10
berth_num = 10
N = 210


def berth_expand(berth_pos):
    x, y = berth_pos
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
        找到领点
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
        self.x = startX  # 机器人当前位置
        self.y = startY
        self.goods = goods  # 0:空 1:满
        self.status = status  # 0:待机故障 1:正常运行
        self.mbx = mbx
        self.mby = mby
        self.u_set = [(-1, 0), (0, 1), (1, 0), (0, -1)]  # 状态的位移点
        self.crash_flag = False  # 是否发生故障标志，为了重启
        self.avoid_flag = False  # 避让标志
        self.wait_total_num = random.randint(0, 3)  # 等待几回合,随机的,每个机器人不一样
        self.wait_num = 0  # 记录等待的回合
        self.minimum_dis = float('inf')  # 记录当前最小货物距离，是全局性的缓存
        self.min_cargo_pos = (1, 2)  # 记录当前最近货物位置
        self.good_get_fix_flag = 0
        self.path_length = 0
        self.last = 0   # 进入最终环节
        self.last_target = 0  # 最终环节目标港口
        self.change_flag = 0  # 改变目标时标志位
        self.path_length = 0  # 记录路径当前已经走的长度
        self.nearest_cargo = None

    def get_neighbor_for_avoid_crash(self, s):
        """
        找到不是障碍的领点
        :param s: 状态
        :return: 邻点
        """

        return [(s[0] + u[0], s[1] + u[1]) for u in self.u_set]


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
        # sys.stderr.write('robot_id:' + str(robot_id)+'\n')
        global path_has_find_flag_for_berth  # 是否已经找到了寻货路径
        global path_has_find_flag_for_cargo  # 是否已经找到了去港口路径
        global robot_goal_axis_save  # 先不写成全局的，每回合要进入
        global robot_current_axis_save  # 机器人的当前位置变量

        # sys.stderr.write('robot_id:' + str(robot_id)+'\n')

        # 状态为0停止运行（随机走）
        if self.status == 0:
            self.crash_flag = True
            path_has_find_flag_for_berth[robot_id] = False
            path_has_find_flag_for_cargo[robot_id] = False
            pass

        # 正常运行
        elif self.status == 1:
            """
            这段还是留着，增加系统鲁棒性
            """
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
                    nearest_cargo = None  # 计算初始化
                    if not path_has_find_flag_for_cargo[robot_id]:
                        max_pervalue = float(0)
                        for cargo_pos in cargo_gds:
                            distance = abs(cargo_pos[0] - current_pos[0]) + abs(cargo_pos[1] - current_pos[1]) + 1
                            value = gds[cargo_pos[0]][cargo_pos[1]]
                            if value / distance > max_pervalue:
                                max_pervalue = value / distance
                                # for i in range(5):
                                #     sys.stderr.write('找到一个更小的' + str(max_pervalue) + '  机器人id:' + str(robot_id) + '\n')
                                nearest_cargo = cargo_pos
                    # if not path_has_find_flag_for_cargo[robot_id]:
                    #     min_distance = float('inf')
                    #     for cargo_pos in cargo_gds:  # 这里计算的是每一回合离自己最近的货物
                    #         distance = abs(cargo_pos[0] - current_pos[0]) + abs(cargo_pos[1] - current_pos[1])
                    #         if distance < min_distance:
                    #             min_distance = distance
                    #             nearest_cargo = cargo_pos

                    if self.avoid_flag:  # 如果避让了，会改变某个标志，然后进入这个循环重新寻路，这个地方要把寻路的flag变为False，把自己的flag改为False
                        path_has_find_flag_for_cargo[robot_id] = False
                        self.avoid_flag = False

                    # 是否已经找到路径，如果没有，进入此循环进行路径查找
                    if not path_has_find_flag_for_cargo[robot_id]:
                        # 向最近的货物位置移动，寻路
                        astar = AStar(current_pos, nearest_cargo, obs_position)
                        path = astar.searching()
                        if not path:  # 如果不存在这样的路径，就删掉。
                            axis.remove(nearest_cargo)  # 删除
                        else:  # 如果存在路径
                            all_robot_path_find_cargo[robot_id] = path  # 意味着上述路径算是找到了
                            new_pos = all_robot_path_find_cargo[robot_id][-2]  # 新的坐标点由路径的[-2]来索引到
                            robot_current_other_pos = set(robot_current_axis_save)
                            robot_current_other_pos.remove(robot_current_axis_save[robot_id])  # 把自己的位置删掉
                            new_obs = set(robot_goal_axis_save) | robot_current_other_pos  # 新的障碍位置
                            if new_pos in new_obs:  # 如果新的位置在新的障碍
                                self.avoid_flag = True  # 下次重新寻路
                                for every_pos in self.get_neighbor_for_avoid_crash(current_pos):  # 这次就遍历邻点，进去移动就行
                                    if every_pos in new_obs:
                                        pass
                                    else:
                                        new_pos = every_pos
                                        robot_goal_axis_save[robot_id] = new_pos
                                        break
                            else:
                                robot_goal_axis_save[robot_id] = new_pos
                            path_has_find_flag_for_cargo[robot_id] = True
                            self.nearest_cargo = nearest_cargo
                            axis.remove(nearest_cargo)
                            all_robot_path_find_cargo[robot_id].pop()  # 因为写到全局，所以要做pop的处理
                            robot_goal_axis_save[robot_id] = new_pos  # 把前面机器人的新位置坐标存进去
                            dr = move_direction(current_pos, new_pos)
                            print("move", robot_id, dr)
                            sys.stdout.flush()
                            # 判断是否到了最优点，到了取货就行取货物
                            if new_pos == nearest_cargo:
                                path_has_find_flag_for_cargo[robot_id] = False
                                print("get", robot_id)
                                sys.stdout.flush()
                                # axis.remove(nearest_cargo)



                    else:  # 如果没有找到更近的货物，直接按照原路线走就行，不用重新规划
                        new_pos = all_robot_path_find_cargo[robot_id][-2]
                        robot_current_other_pos = set(robot_current_axis_save)
                        robot_current_other_pos.remove(robot_current_axis_save[robot_id])  # 把自己的位置删掉
                        new_obs = set(robot_goal_axis_save) | robot_current_other_pos  # 新的障碍位置
                        if new_pos in new_obs:
                            self.avoid_flag = True
                            for every_pos in self.get_neighbor_for_avoid_crash(current_pos):
                                if every_pos in new_obs:
                                    pass
                                else:
                                    new_pos = every_pos
                                    robot_goal_axis_save[robot_id] = new_pos
                                    break
                        else:
                            robot_goal_axis_save[robot_id] = new_pos
                        all_robot_path_find_cargo[robot_id].pop()  # 因为写到全局，所以要做pop的处理
                        robot_goal_axis_save[robot_id] = new_pos  # 把前面机器人的新位置坐标存进去
                        dr = move_direction(current_pos, new_pos)
                        print("move", robot_id, dr)
                        sys.stdout.flush()
                        # 判断是否到了最优点，到了取货就行取货物
                        if new_pos == self.nearest_cargo:  # 使用缓存位置判断
                            path_has_find_flag_for_cargo[robot_id] = False
                            print("get", robot_id)
                            sys.stdout.flush()
                            if self.nearest_cargo in axis:
                                axis.remove(self.nearest_cargo)
                            else:
                                pass


                elif self.goods == 1:  # 有货物，因此找泊位

                    '''使用A*了，这段代码目前作废了'''
                    # if self.avoid_flag:  # 如果避让了，会改变某个标志，然后进入这个循环重新寻路，这个地方要把寻路的flag变为False，把自己的flag改为False
                    #     path_has_find_flag_for_berth[robot_id] = False
                    #     self.avoid_flag = False

                    current_pos = (self.x, self.y)
                    # 直接对应去送对应的港口，不管了
                    if self.last == 0.5:
                        if self.change_flag == 0:
                            self.change_flag = 0.5
                        target_berth = berth_gds[self.last_target]
                        if self.change_flag == 0.5:
                            # for i in range(10):
                            #     sys.stderr.write('机器人编号：' + str(robot_id) + '          0.5555改变目标：' + str(self.last_target) + '\n')
                            path_has_find_flag_for_berth[robot_id] = False
                            self.change_flag = 1
                    #     修改目标，然后设为没有找到路径False
                    elif self.last == 1:
                        if self.change_flag == 0 or self.change_flag == 0.5:
                            self.change_flag = 1.5
                        target_berth = berth_gds[self.last_target]
                        if self.change_flag == 1.5:
                            # for i in range(10):
                                # sys.stderr.write('机器人编号：' + str(robot_id) + '11111改变目标：' + str(self.last_target) + '\n')
                            path_has_find_flag_for_berth[robot_id] = False
                            self.change_flag = 2
                    else:
                        target_berth = berth_gds[robot_id]
                    # 一开始要搜索路径，搜索到后进入else
                    if not path_has_find_flag_for_berth[robot_id]:
                        # 向最近的货物位置移动
                        astar = AStar(current_pos, target_berth, obs_position)
                        path = astar.searching()
                        all_robot_path_go_berth[robot_id] = path
                        new_pos = all_robot_path_go_berth[robot_id][-2]  # 得到下一步位移的坐标
                        robot_current_other_pos = set(robot_current_axis_save)
                        robot_current_other_pos.remove(robot_current_axis_save[robot_id])  # 把自己的位置删掉
                        new_obs = set(robot_goal_axis_save) | robot_current_other_pos  # 新的障碍位置
                        if new_pos in new_obs:
                            obs_new = obs_position | new_obs  # 合并新的障碍位置，重新使用A*计算路径
                            astar = AStar(current_pos, target_berth, obs_new)
                            path = astar.searching()
                            all_robot_path_go_berth[robot_id] = path  # 意味着上述路径算是找到了，并且更改了第一次判断的路径
                            new_pos = all_robot_path_go_berth[robot_id][-2]  # 新的坐标位置
                            robot_goal_axis_save[robot_id] = new_pos  # 存到目标位置列表
                        else:
                            robot_goal_axis_save[robot_id] = new_pos

                        all_robot_path_go_berth[robot_id].pop()  # 因为本回合要移动，所以要pop
                        path_has_find_flag_for_berth[robot_id] = True  # 代表本次找路径搜到了，下次不搜了
                        dr = move_direction(current_pos, new_pos)
                        print("move", robot_id, dr)
                        sys.stdout.flush()
                        # 判断是否到了最优点，到了放货
                        if new_pos in berth_expand(target_berth):
                            if self.last != 0 or zhen >= 11000:
                                for i in range(10):
                                    sys.stderr.write(
                                        '机器人编号：' + str(robot_id) + '   理论到港id：' + str(
                                            self.last_target) + '   实际到港id：' + str(
                                            berth_gds.index(target_berth)) + '    last' + str(self.last) + '\n')
                            path_has_find_flag_for_berth[robot_id] = False  # 到了港口一定要清零标志位
                            print("pull", robot_id)
                            sys.stdout.flush()
                            berth[berth_gds.index(target_berth)].cargo_save += 1

                    else:  # 直接从存好的全局路径中找
                        if self.last != 0:
                            sys.stderr.write(
                                '机器人编号：' + str(robot_id) + '   改变目标：' + str(
                                    self.last_target) + '    last' + str(self.last) + '\n')

                        new_pos = all_robot_path_go_berth[robot_id][-2]
                        robot_current_other_pos = set(robot_current_axis_save)
                        robot_current_other_pos.remove(robot_current_axis_save[robot_id])  # 把自己的位置删掉
                        new_obs = set(robot_goal_axis_save) | robot_current_other_pos  # 新的障碍位置
                        if new_pos in new_obs:
                            obs_new = obs_position | new_obs
                            astar = AStar(current_pos, target_berth, obs_new)
                            path = astar.searching()
                            all_robot_path_go_berth[robot_id] = path
                            new_pos = all_robot_path_go_berth[robot_id][-2]
                            robot_goal_axis_save[robot_id] = new_pos
                        else:
                            robot_goal_axis_save[robot_id] = new_pos
                        all_robot_path_go_berth[robot_id].pop()
                        dr = move_direction(current_pos, new_pos)
                        print("move", robot_id, dr)
                        sys.stdout.flush()
                        if new_pos in berth_expand(target_berth):
                            if self.last != 0 or zhen >= 11000:
                                for i in range(10):
                                    sys.stderr.write(
                                        '机器人编号：' + str(robot_id) + '   理论到港id：' + str(
                                            self.last_target) + '   实际到港id：' + str(
                                            berth_gds.index(target_berth)) + '    last' + str(self.last) + '\n')
                            path_has_find_flag_for_berth[robot_id] = False
                            print("pull", robot_id)
                            sys.stdout.flush()
                            berth[berth_gds.index(target_berth)].cargo_save += 1


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
        self.wait_time = 0  # 已经等待的时间
        self.last_time = 0  # 最后一轮规定等待的时间
        self.first = 1  # 第一次派船
        self.last = 0
        self.go = 0  # 返回港口
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

        if 15000 - 2 * (berth[self.berth_repon[0]].transport_time + berth[self.berth_repon[1]].transport_time) - 200 <= id and self.last == 0:
            for i in range(20):
                sys.stderr.write('倒计时结算：' + str(id) + '    船的id：' + str(boat_id) + '\n')
            # 0 - A - 0 - B - 0
            if self.go == 1 and self.status == 0:
                # 编号为self.berth_repon[self.boat_goal_flag]机器人往港口编号为self.berth_repon[int(not self.boat_goal_flag)]的地方送货
                for i in range(10):
                    sys.stderr.write('去程结算（最后一轮）：' + str(self.berth_repon[self.boat_goal_flag]) + '\n')
                    sys.stderr.write('改变编号的机器人：' + str(
                        self.berth_repon[self.boat_goal_flag]) + '    目标地点:' + str(self.berth_repon[
                                                                                           int(not self.boat_goal_flag)]) + '\n')
                robot[self.berth_repon[int(not self.boat_goal_flag)]].last_target = self.berth_repon[
                    int(not self.boat_goal_flag)]
                robot[self.berth_repon[int(not self.boat_goal_flag)]].last = 1
                robot[self.berth_repon[self.boat_goal_flag]].last_target = self.berth_repon[int(not self.boat_goal_flag)]
                robot[self.berth_repon[self.boat_goal_flag]].last = 1
                self.last = 1
            elif self.go == 0 and self.status == 0:
                for i in range(10):
                    sys.stderr.write('倒数第二轮：' + str(self.berth_repon[self.boat_goal_flag]) + '\n')
                    sys.stderr.write('改变编号的机器人：' + str(self.berth_repon[int(not self.boat_goal_flag)]) + '    目标地点:' + str(self.berth_repon[
                    self.boat_goal_flag]) + '\n')
                # 编号为self.berth_repon[int(not self.boat_goal_flag)]机器人往港口编号为self.berth_repon[self.boat_goal_flag]的地方送货
                robot[self.berth_repon[int(not self.boat_goal_flag)]].last_target = self.berth_repon[
                    self.boat_goal_flag]
                robot[self.berth_repon[int(not self.boat_goal_flag)]].last = 0.5
                robot[self.berth_repon[self.boat_goal_flag]].last_target = self.berth_repon[
                    self.boat_goal_flag]
                robot[self.berth_repon[self.boat_goal_flag]].last = 0.5
                self.last = 0.5
            elif self.pos == -1 and self.status == 1:
                for i in range(10):
                    sys.stderr.write('倒数第二轮：' + str(self.berth_repon[self.boat_goal_flag]) + '\n')
                    sys.stderr.write('改变编号的机器人：' + str(
                        self.berth_repon[self.boat_goal_flag]) + '    目标地点:' + str(self.berth_repon[
                                                                                                    int(not self.boat_goal_flag)]) + '\n')
                robot[self.berth_repon[self.boat_goal_flag]].last_target = self.berth_repon[
                    int(not self.boat_goal_flag)]
                robot[self.berth_repon[self.boat_goal_flag]].last = 0.5
                robot[self.berth_repon[int(not self.boat_goal_flag)]].last_target = self.berth_repon[
                    int(not self.boat_goal_flag)]
                robot[self.berth_repon[int(not self.boat_goal_flag)]].last = 0.5
                # 编号为self.berth_repon[self.boat_goal_flag]机器人往港口编号为self.berth_repon[int(not self.boat_goal_flag)]的地方送货
                self.last = 0.5
            else:
                for i in range(10):
                    sys.stderr.write('港口结算：' + str(self.berth_repon[self.boat_goal_flag]) + '\n')
                    sys.stderr.write('改变编号的机器人：' + str(
                        self.berth_repon[int(not self.boat_goal_flag)]) + '    目标地点:' + str(self.berth_repon[
                                                                                                    self.boat_goal_flag]) + '\n')
                robot[self.berth_repon[int(not self.boat_goal_flag)]].last_target = self.berth_repon[
                    self.boat_goal_flag]
                robot[self.berth_repon[int(not self.boat_goal_flag)]].last = 0.5
                robot[self.berth_repon[self.boat_goal_flag]].last_target = self.berth_repon[
                    self.boat_goal_flag]
                robot[self.berth_repon[self.boat_goal_flag]].last = 0.5
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
                    self.last_time = (15000 - id) - berth[self.berth_repon[self.boat_goal_flag]].transport_time - 2 * berth[
                        self.berth_repon[int(not self.boat_goal_flag)]].transport_time - boat_capacity * berth[self.berth_repon[int(
                        not self.boat_goal_flag)]].loading_speed - 100
                    self.num = min(self.wait_time // berth[self.berth_repon[self.boat_goal_flag]].loading_speed, berth[self.berth_repon[self.boat_goal_flag]].cargo_save)

                    sys.stderr.write('倒数第二轮在港等待：' + str(self.berth_repon[self.boat_goal_flag]) + '剩余等待时间：' + str(self.last_time) + '\n')
                    sys.stderr.write('倒数第二轮离开' + str(
                        berth[self.berth_repon[self.boat_goal_flag]].berth_id) + '    船装的货物' + str(
                        self.num) + '    船id' + str(boat_id) + '    对应机器人目标' + str(
                        robot[self.berth_repon[self.boat_goal_flag]].last_target) + ',' + str(
                        robot[self.berth_repon[int(not self.boat_goal_flag)]].last_target) + '     港口的货物' + str(
                        berth[self.berth_repon[self.boat_goal_flag]].cargo_save) + '  另一港口的货物' + str(
                        berth[self.berth_repon[int(not self.boat_goal_flag)]].cargo_save) + '\n')
                    if 0 >= self.last_time or boat_capacity <= self.num + 1:
                        self.wait_time = 0
                        berth[self.berth_repon[self.boat_goal_flag]].cargo_save = max(0, self.num - berth[self.berth_repon[self.boat_goal_flag]].cargo_save)
                        self.num = 0
                        self.last = 1

                        # 编号为self.berth_repon[self.boat_goal_flag]机器人往港口编号为self.berth_repon[int(not self.boat_goal_flag)]的地方送货
                        robot[self.berth_repon[self.boat_goal_flag]].last_target = self.berth_repon[
                            int(not self.boat_goal_flag)]
                        robot[self.berth_repon[int(not self.boat_goal_flag)]].last_target = self.berth_repon[
                            int(not self.boat_goal_flag)]

                        robot[self.berth_repon[int(not self.boat_goal_flag)]].last = 1
                        robot[self.berth_repon[self.boat_goal_flag]].last = 1
                        for i in range(20):
                            sys.stderr.write('最后一轮跳转结算（最后一轮）：' + str(self.berth_repon[self.boat_goal_flag]) + '\n')
                            sys.stderr.write('改变编号的机器人：' + str(
                                self.berth_repon[self.boat_goal_flag]) + '    目标地点:' + str(self.berth_repon[
                                                                                                   int(not self.boat_goal_flag)]) + '\n')
                        print("go", boat_id)
                        sys.stdout.flush()
                        self.go = 1

                    if self.go != 1:
                        self.wait_time += 1
                elif self.last == 1:
                    self.last_time = (15000 - id) - berth[self.berth_repon[self.boat_goal_flag]].transport_time - 100
                    self.num = min(self.wait_time // berth[self.berth_repon[self.boat_goal_flag]].loading_speed,
                                   berth[self.berth_repon[self.boat_goal_flag]].cargo_save)
                    sys.stderr.write(
                    '最后一轮在港等待：' + str(self.berth_repon[self.boat_goal_flag]) + '剩余等待时间：' + str(
                        self.last_time) + '\n')
                    sys.stderr.write('最后一轮离开' + str(
                        berth[self.berth_repon[self.boat_goal_flag]].berth_id) + '    船装的货物' + str(
                        self.num) + '    船id' + str(boat_id) + '    对应机器人目标' + str(
                        robot[self.berth_repon[self.boat_goal_flag]].last_target) + ',' + str(
                        robot[self.berth_repon[int(not self.boat_goal_flag)]].last_target) + '     港口的货物' + str(
                        berth[self.berth_repon[self.boat_goal_flag]].cargo_save) + '  另一港口的货物' + str(
                        berth[self.berth_repon[int(not self.boat_goal_flag)]].cargo_save) + '\n')
                    if 0 >= self.last_time:
                        # for i in range(10):
                        #     sys.stderr.write('最后一轮离开' + str(
                        #         berth[self.berth_repon[self.boat_goal_flag]].berth_id) + '    船装的货物' + str(
                        #         self.num) + '    船id' + str(boat_id)  + '     港口的货物' + str(
                        #         berth[self.berth_repon[self.boat_goal_flag]].cargo_save) + '  另一港口的货物' + str(berth[self.berth_repon[int(not self.boat_goal_flag)]].cargo_save) + '\n')

                        self.wait_time = 0
                        berth[self.berth_repon[self.boat_goal_flag]].cargo_save = max(0, self.num - berth[self.berth_repon[self.boat_goal_flag]].cargo_save)
                        self.num = 0

                        print("go", boat_id)
                        sys.stdout.flush()
                        self.go = 1
                    if self.go != 1:
                        self.wait_time += 1

                else:
                    self.num = min(self.wait_time // berth[self.berth_repon[self.boat_goal_flag]].loading_speed, berth[self.berth_repon[self.boat_goal_flag]].cargo_save)
                    # sys.stderr.write('倒计时结算：' + str(15000 - 2 * (
                    #             berth[self.berth_repon[0]].transport_time + berth[
                    #         self.berth_repon[1]].transport_time) - 200) + '    船的id：' + str(boat_id) + '\n')
                    sys.stderr.write('船在港口等待' + str(berth[self.berth_repon[self.boat_goal_flag]].berth_id) + '    船装的货物' + str(self.num) + '    船id' + str(boat_id) + '    船的last' + str(self.last) + '     港口的货物' + str(berth[self.berth_repon[self.boat_goal_flag]].cargo_save) + '\n')
                    # sys.stderr.write('船在港口装货的速度' + str(berth[int(not self.boat_goal_flag)].loading_speed)  + '    船等待的时间'  + str(self.wait_time) + '\n')
                    if boat_capacity <= self.num + 1  or (self.num > 9 and berth[self.berth_repon[self.boat_goal_flag]].cargo_save <= self.num) or (self.num < 9 and berth[self.berth_repon[self.boat_goal_flag]].cargo_save <= self.num  and berth[self.berth_repon[int(not self.boat_goal_flag)]].cargo_save >= boat_capacity // 2):  #直接走
                        # if self.num + berth[self.boat_goal_flag].cargo_save
                        self.wait_time = 0
                        berth[self.berth_repon[self.boat_goal_flag]].cargo_save = max(0, berth[self.berth_repon[self.boat_goal_flag]].cargo_save - self.num)
                        self.num = 0

                        print("go", boat_id)
                        sys.stdout.flush()
                        self.go = 1
                    # elif self.num < 9 and berth[self.berth_repon[int(not self.boat_goal_flag)]].cargo_save >= boat_capacity // 2:

                        # self.wait_time = 0
                        # berth[self.berth_repon[self.boat_goal_flag]].cargo_save = max(0, berth[
                        #     self.berth_repon[self.boat_goal_flag]].cargo_save - self.num)
                        # if self.boat_goal_flag:  # 移动到A位
                        #     print("ship", boat_id, self.berth_repon[0])
                        #     sys.stdout.flush()
                        #     self.boat_goal_flag = 0
                        # else:  # 移动到B位
                        #     print("ship", boat_id, self.berth_repon[1])
                        #     sys.stdout.flush()
                        #     self.boat_goal_flag = 1






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

# 基本信息
money = 0
boat_capacity = 0
id = 0
ch = []
gds = [[0 for _ in range(N)] for _ in range(N)]
axis = []  # 货物的坐标点集
cargo_dont_find = set()  # 货物没有找到的集合,留下一个接口后面处理
obs_position = set()  # 障碍点的初始化
berth_axis = []  # 泊位的坐标点集
robot_current_axis_save = [(i, 1) for i in range(0, 10)]  # 机器人当前目标位置变量

# 针对泊位的信息
boat_load_time = [i for i in range(5)]
berth_pair = [0 for _ in range(5)]

# 针对全局路径的list和标志位
robot_path = []
all_robot_path_find_cargo = [robot_path.copy() for _ in range(10)]  # 所有机器人的路径,存在全局里
all_robot_path_go_berth = [robot_path.copy() for _ in range(10)]  # 所有机器人的路径,存在全局里
path_has_find_flag_for_cargo = [False for _ in range(10)]  # 所有机器人找到路径的标志位，默认False，则第一次进入会找路径，找到路径后会置为True
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
    global boat_capacity  # 传入船容量
    boat_capacity = int(input())
    okk = input()
    global obs_position  # 传入障碍位置
    obs_position = obs_map(ch)
    global berth_axis  # 传入泊位坐标
    berth_axis = [(berth[i].x, berth[i].y) for i in range(10)]  # 泊位坐标，还没优化，因为只计算左上角的点
    global berth_pair
    berth_pair = get_berth_pair()
    for i in range(5):
        boat[i].berth_repon = berth_pair[i]
        # sys.stderr.write('船对应港口初始化' + str(boat[i].berth_repon) + '\n')
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
        # 专门为了调试跳出
        # if zhen > 500:
        #     print("move", 40, 20)
        id = Input()

        robot_goal_axis_save = [(0, 0) for _ in range(10)]  # 研究了一下，得每次循环时初始化，起码机器人在一个循环内不变。
        for i in range(robot_num):
            try:
                robot[i].action(i, axis, berth_axis)


            except:
                pass

        sys.stderr.write('4号机器人goods' + str(robot[4].goods) + '  是否找到路径' + str(path_has_find_flag_for_berth[4]) + '\n')
        sys.stderr.write('8号机器人goods' + str(robot[8].goods) + '  是否找到路径' + str(path_has_find_flag_for_berth[8]) +'\n')
        sys.stderr.write('9号机器人goods' + str(robot[9].goods) + '  是否找到路径' + str(path_has_find_flag_for_berth[9]) +'\n')
        for j in range(5):
            try:
                boat[j].action_boat(j)
            except:
                pass
        print("OK")
        sys.stdout.flush()