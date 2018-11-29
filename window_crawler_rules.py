from pyknow import *
import schema
import ProjectAstar
import get_poop
import numpy as np
import tkinter as tk
import time
import threading
from PIL import ImageTk, Image
import os
from keras.models import load_model
import glob


def split(A, num):
    return int(A.split(",")[num])

def a_star(start, end, map, trans):
    global a_calculating
    endx = split(end, 0)
    endy = split(end, 1)
    startx = split(start, 0)
    starty = split(start, 1)
    if endx >= len(map):
        endx = len(map)-1
    if endy >= len(map[0]):
        endy = len(map[0])-1
    path_string = ""
    if utils.unfreeze(map)[endx][endy] is 1.0:
        return "no_route"
    a_calculating = True
    path = ProjectAstar.astar(utils.unfreeze(trans), utils.unfreeze(map),
                              (startx, starty), (endx, endy))
    a_calculating = False
    try:
        for i in range(len(path)):
            if i is 0:
                continue
            path_string += str(path[i][0]) + "," + str(path[i][1]) + "$"
    except:
        return "no_route"
    return path_string[:-1]


class window(Fact):
    state = Field(schema.Or("closed", "open"), default="closed")
    transparency = Field(schema.Or("clean", "dirty"), default="dirty")
    location = Field(str, mandatory=True)
    
class crawler(Fact):
    name = Field(str, mandatory=True)
    location = Field(str, mandatory=True)
    goal = Field(str, mandatory=True)

class route(Fact):
    path = Field(str, mandatory=True)
    end_point = Field(str, mandatory=True)
    world_map = Field(list, mandatory=True)
    transparency_map = Field(list, mandatory=True)
    preplanned_route = Field(list, mandatory=True)

class system_control:
    @Rule(
        AS.craw << crawler(goal=MATCH.goal, location=MATCH.craw_loc,
                           name=MATCH.name),
        AS.rout << route(path=MATCH.path, end_point=MATCH.end_point,
                         world_map=MATCH.map,
                         preplanned_route=MATCH.pre_rou,
                         transparency_map=MATCH.trans),
        TEST(lambda goal : goal is "None"))
    def update_goal_none(self, rout, path, end_point, craw_loc, craw,
                         map, pre_rou, trans, name):
        global drop_route
        if drop_route:
            self.modify(rout, end_point="0,0", path="None",
                        preplanned_route=["going_home"])
            print("Forced to go home")
            drop_route = False
        elif path is "None":
            route = a_star(craw_loc, end_point, map, trans)
            if len(route) is 0:
                if len(utils.unfreeze(pre_rou)) < 1:
                    self.modify(rout, end_point="0,0", path="None",
                                preplanned_route=["going_home"])
                    #print("Going home")
                elif pre_rou[0] is "going_home":
                    self.modify(rout, path=route)
                    print("Ones are dirty windows")
                    print(utils.unfreeze(trans))
                    print("Ones are open windows")
                    print(utils.unfreeze(map))
                    #print("Robot done")
                else:
                    temp = utils.unfreeze(pre_rou)[0]
                    temp2 = utils.unfreeze(pre_rou)[1:]
                    self.modify(rout, end_point=temp, path="None",
                                preplanned_route=temp2)
                    #print("Reached subgoal")
                    
            elif route is "no_route":
                if len(utils.unfreeze(pre_rou)) < 1:
                    self.modify(rout, end_point="0,0", path="None",
                                preplanned_route=["going_home"])
                    #print("Going home")
                elif pre_rou[0] is "going_home":
                    self.modify(rout, path=route)
                    print("Ones are dirty windows")
                    print(utils.unfreeze(trans))
                    print("Ones are open windows")
                    print(utils.unfreeze(map))
                    #print("Robot done")
                else:
                    temp = utils.unfreeze(pre_rou)[0]
                    temp2 = utils.unfreeze(pre_rou)[1:]
                    self.modify(rout, end_point=temp, path="None",
                                preplanned_route=temp2)
                    #print("No route to goal found")
            else:
                self.modify(rout, path=route)
                #print("Calculated path")
        else:
            self.modify(craw, goal=path.split("$",1)[0])
            try:
                path = path.split("$",1)[1]
                self.modify(rout, path=path)
            except:
                self.modify(rout, path="None")

    @Rule(
        AS.craw << crawler(goal=MATCH.goal, location=MATCH.location),
        window(transparency=MATCH.trans, location=MATCH.location),
        TEST(lambda goal :  goal is "take_picture"))
    def take_picture(self, craw, trans, location):
        #print("Taking picture at " + location)
        global wasted_movement
        if trans is "dirty":
            self.modify(craw, goal="clean")
            #print("Window is dirty")
            global get_picture
            wasted_movement -= 1
            get_picture = True
            time.sleep(1)
        else:
            wasted_movement += 1
            self.modify(craw, goal="None")
            #print("Window is clean")

    @Rule(
        AS.craw << crawler(goal=MATCH.goal, location=MATCH.location),
        AS.rout << route(transparency_map=MATCH.tran,
                         preplanned_route=MATCH.pre_rou),
        AS.win << window(transparency=MATCH.trans, location=MATCH.location),
        TEST(lambda goal : goal is "clean"))
    def clean_window(self, craw, win, tran, location, rout, pre_rou):
        global update_dirt, dx, dy
        temp_rou = utils.unfreeze(pre_rou)
        try:
            temp_rou.remove(location)
            #print("Removing " + location + " from route")
        except:
            pass
        self.modify(craw, goal="take_picture")
        self.modify(win, transparency="clean")
        map_tran = utils.unfreeze(tran)
        map_tran[split(location,0)][split(location,1)] = 0.0
        self.modify(rout, transparency_map=map_tran, preplanned_route=temp_rou)
        #print("Cleaning window")  
        update_dirt = True
        dx = split(location, 0)
        dy = split(location, 1)

            
class move:
    @Rule(
        AS.rout << route(world_map=MATCH.map, end_point=MATCH.end_point),
        window(location=MATCH.win_loc, state=MATCH.state),
        AS.craw << crawler(name=MATCH.name, goal=MATCH.goal_loc,
                           location=MATCH.craw_loc),
        TEST(lambda win_loc, goal_loc, craw_loc : win_loc == goal_loc and
             not craw_loc == win_loc))
    def move_to_location(self, win_loc, craw, craw_loc, state, rout, map,
                         end_point, name):
        if state is "open":
            map_temp = utils.unfreeze(map)
            map_temp[split(win_loc,0)][split(win_loc,1)] = 1.0
            self.modify(rout, path="None", world_map=map_temp)
            self.modify(craw, goal="None")
            #print("Window is open. Cannot move to next window")
            global wx, wy, update_window
            wx = split(win_loc, 0)
            wy = split(win_loc, 1)
            update_window = True
        else:
            self.modify(craw, location=win_loc, goal="take_picture")
            print("Moving from " + str(craw_loc) + " to " + str(win_loc) +
                  " End point is: " + str(end_point))
            print("By " +str(name))
            if (abs(split(craw_loc,0) - split(win_loc,0)) > 1 or
                abs(split(craw_loc,1) - split(win_loc,1)) > 1):
                print("ERROR-------------------------------"
                      "------------------------------------")
            global rx, ry, robot_moved, update_robot, steps
            rx = split(win_loc, 0)
            ry = split(win_loc, 1)
            robot_moved = True
            update_robot = True
            steps += 1

class clean_windows(KnowledgeEngine, move, system_control):

    @DefFacts()
    def startup(self):
        global bx
        global by
        global map_draw
        global map_draw_dirt
        preplanned_route = []
        il = by
        ij = bx
        map_temp = np.zeros([il,ij]).tolist()
        map_draw = np.zeros([il,ij]).tolist()
        map_trans = np.ones([il,ij]).tolist()
        map_draw_dirt = map_trans
        yield crawler(name = "mr_roboto_1", location = "-1,-1", goal = "None")
        #yield crawler(name = "mr_roboto_2", location = "0,0", goal = "None")

        for i in range(il):
            for j in range(ij):
                if np.random.randint(0,3) is 1:
                    if i is 0 and j is 0:
                        yield window(location = str(i) + "," + str(j))
                        continue    
                    map_draw[i][j] = 1
                    yield window(location = str(i) + "," + str(j), state="open")
                else:
                    yield window(location = str(i) + "," + str(j))
                if i%2 > 0:
                    preplanned_route.append(str(i)+","+str((ij-1)-j))
                else:
                    preplanned_route.append(str(i)+","+str(j))
        print(map_temp)
        yield route(path = "None", end_point = preplanned_route.pop(0),
                    world_map=map_temp, transparency_map=map_trans, 
                    preplanned_route=preplanned_route)

# ^^^^^ PYKNOW ^^^^
# VVVVV UI VVVVVVVV

class Tkinter:
    def __init__(self, world):
        global bx
        global by
        self.model = load_model("modelfit.h5")
        self.image_paths = glob.glob("image_data/*.jpg")
        self.rx_old = 0
        self.ry_old = 0
        self.stop_simu = False
        self.end_all = False
        self.yellow_on = "#fbff1c"
        self.yellow_off = "#58591e"
        self.green_on = "#69f43f"
        self.green_off = "#1f4912"
        self.red_on = "#e2381d"
        self.red_off = "#4c1911"
        self.world = world
        self.height = 100 * bx
        self.lenght = 100 * by + 350
        self.start_flag = False
        self.button_height = 40
        self.button_lenght = 75
        self.space = 5
        self.top = tk.Tk()
        self.top.title("Window crawler UI")
        self.top_gemometry(self.lenght, self.height, self.top)
        self.message_to_user = tk.StringVar()
        self.message_to_user.set("Status")
        self.message_to_user_bsize = tk.StringVar()
        self.message_to_user_bsize.set("Building Size:")
        self.reset_simu = False
        self.message_to_user_bsizexy = tk.StringVar()
        self.message_to_user_bsizexy.set(str(bx)+" x "+str(by))
        self.message_to_user_rpos = tk.StringVar()
        self.message_to_user_rpos.set("Robot Pos:")
        self.message_to_user_miss = tk.StringVar()
        self.message_to_user_miss.set("Walked on clean windows: 0")
        self.message_to_user_rposxy = tk.StringVar()
        self.message_to_user_rposxy.set("("+str(0)+","+str(0)+")")
        self.message_to_user_lp = tk.StringVar()
        self.message_to_user_lp.set("Last Picture:")
        self.message_to_user_pre = tk.StringVar()
        self.message_to_user_pre.set("Predicted:")
        self.message_to_user_preres = tk.StringVar()
        self.message_to_user_preres.set("None")
        self.button_text_1 = tk.StringVar()
        self.button_text_1.set("Start")
        self.button_text_2 = tk.StringVar()
        self.button_text_2.set("Stop")
        self.button_text_3 = tk.StringVar()
        self.button_text_3.set("Return home")
        self.button_text_4 = tk.StringVar()
        self.button_text_4.set("Reset Simulation")
        self.button_text_5 = tk.StringVar()
        self.button_text_5.set("Stop Simulation")
        self.button_text_6 = tk.StringVar()
        self.button_text_6.set("-")
        self.button_text_7 = tk.StringVar()
        self.button_text_7.set("+")
        self.button_text_8 = tk.StringVar()
        self.button_text_8.set("-")
        self.button_text_9 = tk.StringVar()
        self.button_text_9.set("+")
        self.label = tk.Label(textvariable=self.message_to_user)
        self.labelbsize = tk.Label(textvariable=self.message_to_user_bsize)
        self.labelbsizexy = tk.Label(textvariable=self.message_to_user_bsizexy)
        self.labelrpos = tk.Label(textvariable=self.message_to_user_rpos,
                                  justify=tk.LEFT)
        self.labelmiss = tk.Label(textvariable=self.message_to_user_miss,
                                  justify=tk.LEFT)
        self.labelrposxy = tk.Label(textvariable=self.message_to_user_rposxy)
        self.labelimg = tk.Label(image='')
        self.labellp = tk.Label(textvariable=self.message_to_user_lp)
        self.labelpre = tk.Label(textvariable=self.message_to_user_pre)
        self.labelpreres = tk.Label(textvariable=self.message_to_user_preres)
        self.B1 = tk.Button(textvariable=self.button_text_1,
                            command=lambda: self.start())
        self.B2 = tk.Button(textvariable=self.button_text_2,
                            command=lambda: self.stop())
        self.B3 = tk.Button(textvariable=self.button_text_3,
                            command=lambda: self.go_home())
        self.B4 = tk.Button(textvariable=self.button_text_4,
                            command=lambda: self.reset_sim())
        self.B5 = tk.Button(textvariable=self.button_text_5,
                            command=lambda: self.stop_sim())
        self.B6 = tk.Button(textvariable=self.button_text_6,
                            command=lambda: self.dec_x())
        self.B7 = tk.Button(textvariable=self.button_text_7,
                            command=lambda: self.inc_x())
        self.B8 = tk.Button(textvariable=self.button_text_8,
                            command=lambda: self.dec_y())
        self.B9 = tk.Button(textvariable=self.button_text_9,
                            command=lambda: self.inc_y())
        self.box_green = tk.Text(background=self.green_off)
        self.box_red = tk.Text(background=self.red_on)
        self.box_yellow = tk.Text(background=self.yellow_off)
        self.canvas = tk.Canvas()

    def update_tkinter(self):
        global bx, by
        self.height = 30 * by
        self.lenght = 30 * bx + 450
        if self.height < 320:
            self.height = 320
        if self.lenght < 570:
            self.lenght = 570
        self.labelimg.config(image='')
        self.labelimg = tk.Label(image='')
        self.message_to_user_preres.set("None")
        self.top_gemometry(self.lenght, self.height, self.top)
        self.canvas.place(bordermode=tk.OUTSIDE, height=30 * by,
                          width=30 * bx, x=self.lenght-(self.button_lenght*2+30*
                          bx*1+15*self.space),
                          y=self.height-30*by)
        self.box_yellow.place(bordermode=tk.OUTSIDE, height=self.button_height/2,
                              width=self.button_lenght/4,
                              x=self.lenght-self.button_lenght+self.button_lenght/4*
                              2+self.space, y=self.space+self.button_height/4)
        self.box_red.place(bordermode=tk.OUTSIDE, height=self.button_height/2,
                           width=self.button_lenght/4,
                           x=self.lenght-self.button_lenght+self.button_lenght/4,
                           y=self.space+self.button_height/4)
        self.box_green.place(bordermode=tk.OUTSIDE, height=self.button_height/2,
                             width=self.button_lenght/4,
                             x=self.lenght-self.space-self.button_lenght,
                             y=self.space+self.button_height/4)
        self.B9.place(bordermode=tk.OUTSIDE, height=self.space*3,
                      width=self.space*3,
                      x=self.space*10+self.button_lenght,
                      y=self.button_height-self.space)
        self.B8.place(bordermode=tk.OUTSIDE, height=self.space*3,
                      width=self.space*3,
                      x=self.button_lenght+self.space*7,
                      y=self.button_height-self.space)
        self.B7.place(bordermode=tk.OUTSIDE, height=self.space*3,
                      width=self.space*3,
                      x=self.space*1+self.button_lenght,
                      y=self.button_height-self.space)
        self.B6.place(bordermode=tk.OUTSIDE, height=self.space*3,
                      width=self.space*3,
                      x=self.button_lenght-self.space*2,
                      y=self.button_height-self.space)
        self.B5.place(bordermode=tk.OUTSIDE, height=self.button_height,
                      width=self.button_lenght*2+self.space,
                      x=self.lenght-self.space*2-self.button_lenght*2,
                      y=self.height-self.space*2-self.button_height)
        self.B4.place(bordermode=tk.OUTSIDE, height=self.button_height,
                      width=self.button_lenght*2+self.space,
                      x=self.lenght-self.space*2-self.button_lenght*2,
                      y=self.height-self.space*3-self.button_height*2)
        self.B3.place(bordermode=tk.OUTSIDE, height=self.button_height,
                      width=self.button_lenght*2+self.space,
                      x=self.lenght-self.space*2-self.button_lenght*2,
                      y=self.space*3+self.button_height*2)
        self.B2.place(bordermode=tk.OUTSIDE, height=self.button_height,
                      width=self.button_lenght,
                      x=self.lenght-self.space-self.button_lenght,
                      y=self.space*2+self.button_height)
        self.B1.place(bordermode=tk.OUTSIDE, height=self.button_height,
                      width=self.button_lenght,
                      x=self.lenght-self.space*2-self.button_lenght*2,
                      y=self.space*2+self.button_height)
        self.labelpreres.place(bordermode=tk.OUTSIDE, height=self.button_height,
                               width=self.button_lenght,
                               x=self.space*2+self.button_lenght,
                               y=self.height-self.space-self.button_height)
        self.labelpre.place(bordermode=tk.OUTSIDE, height=self.button_height,
                            width=self.button_lenght,
                            x=self.space,
                            y=self.height-self.space-self.button_height)
        self.labelimg.place(bordermode=tk.OUTSIDE, height=self.button_height*3,
                            width=self.button_lenght*2,
                            x=self.space,
                            y=self.height-self.space*2-self.button_height*4)
        self.labellp.place(bordermode=tk.OUTSIDE, height=self.button_height,
                           width=self.button_lenght,
                           x=self.space,
                           y=self.height-self.space*3-self.button_height*5)
        self.labelrposxy.place(bordermode=tk.OUTSIDE, height=self.button_height,
                               width=self.button_lenght/2,
                               x=self.space+self.button_lenght,
                               y=self.space*2+self.button_height)
        self.labelrpos.place(bordermode=tk.OUTSIDE, height=self.button_height,
                             width=self.button_lenght,
                             x=self.space,
                             y=self.space*2+self.button_height)
        self.labelmiss.place(bordermode=tk.OUTSIDE, height=self.button_height/2,
                             width=self.button_lenght*2+self.space*6,
                             x=self.space*2,
                             y=self.space+self.button_height*2)
        self.labelbsizexy.place(bordermode=tk.OUTSIDE, height=self.button_height,
                                width=self.button_lenght/2,
                                x=self.space+self.button_lenght,
                                y=self.space)
        self.labelbsize.place(bordermode=tk.OUTSIDE, height=self.button_height,
                              width=self.button_lenght,
                              x=self.space,
                              y=self.space)
        self.label.place(bordermode=tk.OUTSIDE, height=self.button_height,
                         width=self.button_lenght,
                         x=self.lenght-self.space*2-self.button_lenght*2,
                         y=self.space)

    def top_gemometry(self, w, h, top):
        ws = top.winfo_screenwidth()
        hs = top.winfo_screenheight()
        # Start middle
        x = (ws/2) - (w/2)
        y = (hs/2) - (h/2)
        # Start corner
        # x = ws-w
        # y = 0
        top.geometry('%dx%d+%d+%d' % (w, h, x, y))

    def start(self):
        self.box_green.config(background=self.green_on)
        self.box_red.config(background=self.red_off)
        self.start_flag = True

    def stop(self):
        self.box_green.config(background=self.green_off)
        self.box_red.config(background=self.red_on)
        self.start_flag = False

    def go_home(self):
        global drop_route
        drop_route = True

    def reset_sim(self):
        self.reset_simu = True

    def stop_sim(self):
        self.top.destroy()
        self.stop_simu = True

    def dec_x(self):
        global bx
        global by
        if bx > 2:
            bx = bx - 1
        self.message_to_user_bsizexy.set(str(bx)+" x "+str(by))

    def inc_x(self):
        global bx
        global by
        if bx < 20:
            bx = bx + 1
        self.message_to_user_bsizexy.set(str(bx)+" x "+str(by))

    def dec_y(self):
        global bx
        global by
        if by > 2:
            by = by - 1
        self.message_to_user_bsizexy.set(str(bx)+" x "+str(by))

    def inc_y(self):
        global bx
        global by
        if by < 20:
            by = by + 1
        self.message_to_user_bsizexy.set(str(bx)+" x "+str(by))

    def init_map(self):
        global map_draw, map_draw_dirt, ry, rx, bx, by
        self.canvas.delete("all")
        size = np.shape(map_draw)
        self.x_init = 30 * bx/2 - bx * 30/2
        self.y_init = 30 * by - by * 30
        x = self.x_init
        y = self.y_init
        w = size[1] * 30 + x
        h = size[0] * 30 + y
        coord_building = x, y, w, h
        building = self.canvas.create_rectangle(coord_building, fill="#4c4a4a")

        self.windows_size = (((w - x) / (size[1])) / 2)

        for row in range(size[0]):
            for i in range(size[1]):
                x = self.x_init + (i * self.windows_size * 2) + 6
                y = self.y_init + (row * self.windows_size * 2) + 6
                w = self.windows_size + x
                h = self.windows_size + y

                coord_window = x, y, w, h

                status_window = map_draw[row][i]
                status_dirt = map_draw_dirt[row][i]

                if status_window == 1:
                    # for making the other line correctly
                    coord_window_reversed = w, y, x, h
                    if status_dirt == 1:
                        window = self.canvas.create_rectangle(coord_window,
                                                              fill="saddlebrown")
                    elif status_dirt == 0:
                        window = self.canvas.create_rectangle(coord_window,
                                                              fill="white")
                    window = self.canvas.create_line(coord_window, fill="red",
                                                     width = 3)
                    window = self.canvas.create_line(coord_window_reversed,
                                                     fill="red", width = 3)
                    #00f6ff
                elif status_window == 0:
                    if status_dirt == 1:
                        window = self.canvas.create_rectangle(coord_window,
                                                              fill="saddlebrown")
                    else:
                        window = self.canvas.create_rectangle(coord_window,
                                                              fill="white")
                if rx is row and ry is i:
                    windows = self.canvas.create_oval(coord_window,
                                                      fill="yellow")

    def  map_update(self):
        global rx, ry, dx, dy, wx, wy, update_robot, update_dirt, update_window
        if update_robot:
            x = self.x_init + (self.ry_old * self.windows_size * 2) + 6
            y = self.y_init + (self.rx_old * self.windows_size * 2) + 6
            w = self.windows_size + x
            h = self.windows_size + y
            coord_window = x, y, w, h
            windows = self.canvas.create_rectangle(coord_window, fill="white")
            self.rx_old = rx
            self.ry_old = ry
            x = self.x_init + (ry * self.windows_size * 2) + 6
            y = self.y_init + (rx * self.windows_size * 2) + 6
            w = self.windows_size + x
            h = self.windows_size + y
            coord_window = x, y, w, h
            windows = self.canvas.create_oval(coord_window, fill="yellow")
            update_robot = False
        if update_dirt:
            x = self.x_init + (dy * self.windows_size * 2) + 6
            y = self.y_init + (dx * self.windows_size * 2) + 6
            w = self.windows_size + x
            h = self.windows_size + y
            coord_window = x, y, w, h
            windows = self.canvas.create_rectangle(coord_window, fill="white")
            x = self.x_init + (ry * self.windows_size * 2) + 6
            y = self.y_init + (rx * self.windows_size * 2) + 6
            w = self.windows_size + x
            h = self.windows_size + y
            coord_window = x, y, w, h
            windows = self.canvas.create_oval(coord_window, fill="yellow")
            update_dirt = False
        if update_window:
            x = self.x_init + (wy * self.windows_size * 2) + 6
            y = self.y_init + (wx * self.windows_size * 2) + 6
            w = self.windows_size + x
            h = self.windows_size + y
            coord_window = x, y, w, h
            coord_window_reversed = w, y, x, h
            window = self.canvas.create_line(coord_window, fill="#04ff00",
                                             width = 3)
            window = self.canvas.create_line(coord_window_reversed,
                                             fill="#04ff00", width = 3)
            update_window = False

    def update_picture(self):
        global get_picture
        if get_picture:
            n_images = len(self.image_paths)
            ran = np.random.randint(0,n_images)
            load = Image.open(self.image_paths[ran])
            resized = load.resize((128, 128),Image.ANTIALIAS)
            render = ImageTk.PhotoImage(resized)
            self.labelimg = tk.Label(image=render)
            self.labelimg.image = render # Keep a reference
            self.labelimg.place(bordermode=tk.OUTSIDE, height=self.button_height*3,
                                width=self.button_lenght*2,
                                x=self.space,
                                y=self.height-self.space*2-self.button_height*4)
            poop = get_poop.get_poop(self.model, ran, self.image_paths)
            if poop is 1:
                self.message_to_user_preres.set("Birdpoop")
            else:
                self.message_to_user_preres.set("No birdpoop")

            get_picture =  False


class run_world (threading.Thread):
    def __init__(self, world, Tkinter):
        threading.Thread.__init__(self)
        self.Tkinter = Tkinter
        self.world = world

    def run(self):
        while True:
            time.sleep(0.01)
            if Tkinter.start_flag:
                world.run(1)
            if Tkinter.end_all:
                break



# Globals:  :(
a_calculating = False
drop_route = False
bx = 10
by = 10
robot_moved = False
rx = 0
ry = 0
map_draw = []
map_draw_dirt = []
dx = 0
dy = 0
update_dirt = False
update_robot = False
update_window = False
wx = 0
wy = 0
get_picture = False
wasted_movement = 0
steps = 0

# pyknow init
world = clean_windows()

Tkinter = Tkinter(world)
Tkinter.rx_old = rx
Tkinter.ry_old = ry

while True:
    steps = 0
    old_move = 0
    wasted_movement = 0
    Tkinter.update_tkinter()
    rx = 0
    ry = 0
    dx = 0
    dy = 0
    wx = 0
    wy = 0
    update_dirt = False
    update_robot = False
    update_window = False
    get_picture = False
    Tkinter.rx_old = rx
    Tkinter.ry_old = ry
    Tkinter.end_all = False
    Tkinter.reset_simu = False
    world.reset()
    Tkinter.init_map()
    Tkinter.world = world
    thread = run_world(world, Tkinter)
    thread.start()
    # Thread

    while True:
        if old_move < wasted_movement:
            old_move = wasted_movement
        Tkinter.message_to_user_miss.set("Walked on clean windows: " +
                                         str(old_move)+ " " +
                                         str(steps))
        Tkinter.update_picture()
        Tkinter.map_update()
        try:
            Tkinter.top.update()
            if a_calculating:
                Tkinter.box_yellow.config(background=Tkinter.yellow_on)
            else:
                Tkinter.box_yellow.config(background=Tkinter.yellow_off)
            if robot_moved:
                Tkinter.message_to_user_rposxy.set("("+str(rx)+","+str(ry)+")")
                robot_moved = False
        except:
            end_all = True
            Tkinter.stop_simu = True
            Tkinter.end_all = True
            break

        if Tkinter.reset_simu:
            Tkinter.end_all = True
            break

    thread.join()

    if Tkinter.stop_simu:
        break




