from pyknow import *
import schema
import ProjectAstar
import numpy as np
import tkinter as tk
import time
import threading


def test_dist(A, B, num):
    return abs(int(A.split(",")[num])-int(B.split(",")[num])) > 1

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
        AS.craw << crawler(goal=MATCH.goal, location=MATCH.craw_loc, name=MATCH.name),
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
        if trans is "dirty":
            self.modify(craw, goal="clean")
            #print("Window is dirty")
        else:
            self.modify(craw, goal="None")
            #print("Window is clean")

    @Rule(
        AS.craw << crawler(goal=MATCH.goal, location=MATCH.location),
        AS.rout << route(transparency_map=MATCH.tran, preplanned_route=MATCH.pre_rou),
        AS.win << window(transparency=MATCH.trans, location=MATCH.location),
        TEST(lambda goal : goal is "clean"))
    def clean_window(self, craw, win, tran, location, rout, pre_rou):
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

            
class move:
    @Rule(
        AS.rout << route(world_map=MATCH.map, end_point=MATCH.end_point),
        window(location=MATCH.win_loc, state=MATCH.state),
        AS.craw << crawler(goal=MATCH.goal_loc, location=MATCH.craw_loc),
        TEST(lambda win_loc, goal_loc, craw_loc : win_loc == goal_loc and
             not craw_loc == win_loc))
    def move_to_location(self, win_loc, craw, craw_loc, state, rout, map, end_point):
        if state is "open":
            map_temp = utils.unfreeze(map)
            map_temp[split(win_loc,0)][split(win_loc,1)] = 1.0
            self.modify(rout, path="None", world_map=map_temp)
            self.modify(craw, goal="None")
            #print("Window is open. Cannot move to next window")
        else:
            self.modify(craw, location=win_loc, goal="take_picture")
            print("Moving from " + str(craw_loc) + " to " + str(win_loc) +
                  "End point is: " + str(end_point))

class clean_windows(KnowledgeEngine, move, system_control):

    @DefFacts()
    def startup(self):
        preplanned_route = []
        il = 10
        ij = 10
        map_temp = np.zeros([il,ij]).tolist()
        map_trans = np.ones([il,ij]).tolist()
        yield crawler(name = "mr_roboto", location = "0,0", goal = "None")

        for i in range(il):
            for j in range(ij):
                if np.random.randint(0,6) is 1:
                    yield window(location = str(i) + "," + str(j), state="open")
                else:
                    yield window(location = str(i) + "," + str(j))
                if i%2 > 0:
                    preplanned_route.append(str(i)+","+str((ij-1)-j))
                else:
                    preplanned_route.append(str(i)+","+str(j))
        yield route(path = "None", end_point = preplanned_route.pop(0),
                    world_map=map_temp, transparency_map=map_trans, 
                    preplanned_route=preplanned_route)



class Tkinter:
    def __init__(self, world):
        self.yellow_on = "#fbff1c"
        self.yellow_off = "#58591e"
        self.green_on = "#69f43f"
        self.green_off = "#1f4912"
        self.red_on = "#e2381d"
        self.red_off = "#4c1911"
        self.end_all = False
        self.world = world
        self.height = 300
        self.lenght = 400
        self.start_flag = False
        self.button_height = 40
        self.button_lenght = 75
        self.space = 5
        self.top = tk.Tk()
        self.top.title("UI")
        self.top_gemometry(self.lenght, self.height, self.top)
        self.message_to_user = tk.StringVar()
        self.message_to_user.set("Status")
        self.button_text_1 = tk.StringVar()
        self.button_text_1.set("Start")
        self.button_text_2 = tk.StringVar()
        self.button_text_2.set("Stop")
        self.button_text_3 = tk.StringVar()
        self.button_text_3.set("Return home")
        self.label = tk.Label(textvariable=self.message_to_user)
        self.label.place(bordermode=tk.OUTSIDE, height=self.button_height,
                         width=self.button_lenght,
                         x=self.lenght-self.space*2-self.button_lenght*2,
                         y=self.space)
        self.B1 = tk.Button(textvariable=self.button_text_1,
                            command=lambda: self.start())
        self.B1.place(bordermode=tk.OUTSIDE, height=self.button_height,
                         width=self.button_lenght,
                         x=self.lenght-self.space*2-self.button_lenght*2,
                         y=self.space*2+self.button_height)
        self.B2 = tk.Button(textvariable=self.button_text_2,
                            command=lambda: self.stop())
        self.B2.place(bordermode=tk.OUTSIDE, height=self.button_height,
                         width=self.button_lenght,
                         x=self.lenght-self.space-self.button_lenght,
                         y=self.space*2+self.button_height)
        self.B3 = tk.Button(textvariable=self.button_text_3,
                            command=lambda: self.go_home())
        self.B3.place(bordermode=tk.OUTSIDE, height=self.button_height,
                         width=self.button_lenght*2+self.space,
                         x=self.lenght-self.space*2-self.button_lenght*2,
                         y=self.space*3+self.button_height*2)
        self.box_green = tk.Text(background=self.green_off)
        self.box_green.place(bordermode=tk.OUTSIDE, height=self.button_height/2,
                         width=self.button_lenght/4,
                         x=self.lenght-self.space-self.button_lenght,
                         y=self.space+self.button_height/4)
        self.box_red = tk.Text(background=self.red_on)
        self.box_red.place(bordermode=tk.OUTSIDE, height=self.button_height/2,
                         width=self.button_lenght/4,
                         x=self.lenght-self.button_lenght+self.button_lenght/4,
                         y=self.space+self.button_height/4)
        self.box_yellow = tk.Text(background=self.yellow_off)
        self.box_yellow.place(bordermode=tk.OUTSIDE, height=self.button_height/2,
                         width=self.button_lenght/4,
                         x=self.lenght-self.button_lenght+self.button_lenght/4*2+
                         self.space, y=self.space+self.button_height/4)

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
        '''
        i = 0
        j = 0
        while True:
            if i >= len(self.world.facts):
                break
            try:
                print(self.world.facts[j])
                i += 1
            except:
                pass
            j += 1
        '''
        #print(len(self.world.facts))
        #print(self.world.facts)
        #self.world.modify(self.world.facts[1], name="BobbyB")
        #print(self.world.facts)

class run_world (threading.Thread):
    def __init__(self, world, Tkinter):
        threading.Thread.__init__(self)
        self.Tkinter = Tkinter
        self.world = world

    def run(self):
        while True:
            #time.sleep(0.1)
            if Tkinter.start_flag:
                world.run(1)
            if Tkinter.end_all:
                break


# Globals:  :(
a_calculating = False
drop_route = False

# pyknow init
world = clean_windows()
world.reset()

Tkinter = Tkinter(world)

# Thread
thread = run_world(world, Tkinter)
thread.start()



while True:
    time.sleep(0.1)
    try:
        Tkinter.top.update()
        if a_calculating:
            Tkinter.box_yellow.config(background=Tkinter.yellow_on)
        else:
            Tkinter.box_yellow.config(background=Tkinter.yellow_off)
    except:
        Tkinter.end_all = True
        break
   


thread.join()



    