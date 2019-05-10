## ANA* Algorithm

# import libraries
from sys import version_info
if version_info.major == 2:
    # We are using Python 2.x
    from Tkinter import *
    import ttk
elif version_info.major == 3:
    # We are using Python 3.x
    from tkinter import *
    from tkinter import ttk

import time as t
import numpy as np
import math
 
import time



'''
Define the color scheme for visualization. You may change it but I recommend using the same colors
'''
# white (0) is an unvisited node, black(1) is a wall, blue(2) is a visited node
# yellow(3) is for start node, green(4) is for exit node, red (5) is a node on the completed path
colors = {5: "red", 4: "green", 3: "yellow", 2: "blue", 1: "black", 0: "white"}

'''
Opens the maze file and creates tkinter GUI object
'''
# load maze
with open("custom.txt") as text:
    maze = [list(line.strip()) for line in text]

    # got the overall size of the maze 
[col, row] = np.shape(maze)


# experimental
parent = []


# create map
root = Tk()
# size of the bof in proportion to row
size = 800 / row

# see width formulae, every time the maze size will be same 
canvas = Canvas(root, width=(size*row), height=(size*col))
# time pass 
root.title("ANA* Algorithm")

def getKey(item):
    return item[3]

# wow
def draw_canvas(canvas, maze):
   
    for i in range(0, col):
        for j in range(0, row):
            # need to understand the last color part
            canvas.create_rectangle(j*size, i*size, (j+1)*size, (i+1)*size, fill=colors[int(maze[i][j])])
            
    canvas.pack()

def is_valid_node(check_row,check_column):

    if (check_column > col-1) or (check_row > row-1) :
        # print("invalid")
        return False
    elif (check_column < 0) or (check_row < 0) :
        # print("invalid")
        return False
    else:
        # print("valid")
        return True
# _____________________________________________     
    

def is_occupied_node(check_row,check_column):

    if ((int(maze[check_row][check_column]) == 1) or (int(maze[check_row][check_column]) == 2)):
        # print("wall")
        return True
    else:
        # print("no wall")
        return False    
#

def is_goal(check_row,check_column):

    if ((int(maze[check_row][check_column]) == 0) and (int(maze[check_row][check_column]) == 14)):
        # print("wall")
        return True
    else:
        # print("no wall")
        return False   

def find_valid_child_nodes(query_row,query_column, g_val):

    temp = []
    # check for adjacent nodes 

    # check for top node
    top_row, top_col = query_row-1,query_column 
    if (is_valid_node(top_row,top_col) == True):
        if(is_occupied_node(top_row,top_col) == False):
            temp.append([top_row,top_col,g_val+1])
            # if it is not visited change the colour
            maze[top_row][top_col] = 2  
            parent.append([top_row, top_col,query_row,query_column])
            
    # check for bottom 
    bottom_row, bottom_col = query_row+1,query_column 
    if (is_valid_node(bottom_row,bottom_col) == True):
        if(is_occupied_node(bottom_row,bottom_col) == False):
            temp.append([bottom_row,bottom_col,g_val+1])
            maze[bottom_row][bottom_col] = 2
            parent.append([bottom_row, bottom_col,query_row,query_column])

    # check for left 
    left_row, left_col = query_row,query_column-1 
    if (is_valid_node(left_row,left_col) == True):
        if(is_occupied_node(left_row,left_col) == False):
            temp.append([left_row,left_col,g_val+1])
            maze[left_row][left_col] = 2
            parent.append([left_row, left_col,query_row,query_column])
            
    # check for right 
    right_row, right_col = query_row,query_column+1 
    if (is_valid_node(right_row,right_col) == True):
        if(is_occupied_node(right_row,right_col) == False):
            temp.append([right_row,right_col,g_val+1])
            maze[right_row][right_col] = 2
            parent.append([right_row, right_col,query_row,query_column])
            
    return temp

def backtrack(data,start_point_row,start_point_col,goal_point_row,goal_point_column):

    goal_row = goal_point_row
    goal_column = goal_point_column
    i = 500
    global parent   
    data = parent
    # for data visulaization only
    # for node in data:
    #     # print(node)

    maze[goal_row][goal_column] = 5 
    complete = False
    while(i>0):
        for node in data:
            # print(node)
            # print(node[0])
            if(goal_row == start_point_row  and goal_column== start_point_col):
                print("backtrakig complete!!")
                complete = True
                maze[start_point_row][start_point_col] = 5
                break 


            if(node[0]==goal_row and node[1]==goal_column):
                goal_row = node[2]
                goal_column = node[3]
                # print("next goal")
                # print(node[3])
                # print(node[4])
                maze[node[2]][node[3]] = 5 
                # input("Press Enter to continue...")
        if complete == True:
            break    

        i = i-1
        # print(i)
        # draw_canvas(canvas, maze)
        # root.update()
    maze[start_point_row][start_point_col] = 5
    draw_canvas(canvas, maze)
    root.update()
    input("Press Enter to proceed further")
                        
    return 50-i

def iter(child_nodes,maze, start_node, exit_node):
    global g_best
    global parent
    i = 1
    init_time = time.time()#int(round(time.time() * 1000))
    while(1):

        # print("current child nodes")
        new_nodes = find_valid_child_nodes(child_nodes[0][0],child_nodes[0][1],child_nodes[0][2])
        # new_nodes = find_valid_child_nodes(child_nodes[0][0],child_nodes[0][1],child_nodes[0][2])
        
            # print(new_nodes)

        if (len(new_nodes)==0):
            # print("blocked node @ " + str(child_nodes[0][0]) + str(child_nodes[0][1]))
            # remove the blocked node from the list 
            child_nodes = child_nodes[1:]
        else:
            i= i+1
            for node in new_nodes:
                # calculate heuristic distances
                dist_x, dist_y = abs(exit_node[0]-node[0]), abs(exit_node[1]-node[1])  
                h = math.sqrt( dist_x*dist_x + dist_y*dist_y)
                g = child_nodes[0][2]
                # print(g)
                # input("Press Enter to continue...")
                    
                # e = g                           #for A* algorithm

                e = (g_best - g) / (h+0.0001)       #for ANA*
                # print(exit_node[1])
                node.append(e)
                node.append(child_nodes[0][0])
                node.append(child_nodes[0][1])
                
                if(node[0]==exit_node[0] and node[1]==exit_node[1]):
                    draw_canvas(canvas, maze)
                    root.update()
                    print("YEAH...GOAL FOUND!")
                    final_time = time.time()
                    input("Press Enter to show current best path searched")
                    backtrack(parent,start_node[0],start_node[1],exit_node[0],exit_node[1])
                    print("Current Best solution is updated to " + str(g))
                    # g_goal = g
                   
                    g_best = g
                    
                    del child_nodes
                    del new_nodes
                    parent = []
                    # new_nodes
                    maze[start_node[0]][start_node[1]] = 2
                    draw_canvas(canvas, maze)
                    root.update()
                    # g=0
                    h=0
                    # ana_star(maze, entrance_node, exit_node,g)
                    #int(round(time.time() * 1000))
                    print("Total time taken in this run = " + str(final_time-init_time))
                    return            
                    # print(child_nodes)
                    # print(parent)
                    

                    

            child_nodes = new_nodes + child_nodes
            

            child_nodes =   sorted (child_nodes, key=getKey)
            child_nodes = child_nodes[::-1]                    # in case of ANA*


    # input("Press Enter to continue...")
    # draw_canvas(canvas, maze)
    # root.update()


# algorithm node
# def ana_star(maze, start_node, exit_node):

def ana_star(start_node, exit_node):

    global g_best
    global maze
    g_incoming = g_best
    # print("g_incoming = "+ str(g_incoming))
    while(1):

        # run the ana_star algorithm
        text = open("custom.txt")
        maze = [list(line.strip()) for line in text]
        [col, row] = np.shape(maze)
        entrance_node = (row+5, 1)
        exit_node = (0, col-10)
        print(exit_node)
        for i in range(0, col):
            for j in range(0, row):
                # need to understand the last color part
                if int(maze[i][j]) > 0:
                    maze[i][j] = 1
                # canvas.create_rectangle(j*size, i*size, (j+1)*size, (i+1)*size, fill=colors[int(maze[i][j])])

        # canvas.pack()
        draw_canvas(canvas, maze)
        root.update()
        # input("Press Enter to proceed further")


        
        # This visualizes the grid. You may remove this and use the functions as you wish.
        # print(start_node[0])
        # print(start_node[1])
        
        maze[start_node[0]][start_node[1]] = 3
        maze[exit_node[0]][exit_node[1]] = 4
        # maze[start_node[0]][start_node[1]] = 3
        
        # input("Press Enter to proceed further")
                        

        # _____________do code debug here________________________
        # maze[15][15] = 5

        # print(exit_node[0])
        # print(exit_node[1])
        # print(row)
        # print(col)

        # print(is_valid_node(0,14))

        # # to check the occupancy
        # print(is_occupied_node(0,14))
        
        
        #-------------------------------------------ANA* Implementation--------------------------------------------------

        # now find the valid child nodes of first node
        
        init_child_nodes = find_valid_child_nodes(start_node[0],start_node[1],0)
       
        for node in init_child_nodes:
                # calculate heuristic distances
                dist_x, dist_y = abs(exit_node[0]-node[0]), abs(exit_node[1]-node[1])  
                h = math.sqrt( dist_x*dist_x + dist_y*dist_y)
                # print(exit_node[1])        
                node.append(0)
                node.append(h)
                
        # print(init_child_nodes)            
        #-----------------------------------------------------------------------------------------------------------
        
        
        child_nodes = init_child_nodes
        # print("g_incoming before main loop " + str(g_incoming))
        # current_best_g =  161
        # print("current_best_g" +str(current_best_g))
        # g_incoming = 103
        # print("g incoming")
        # print(g_incoming)

        # print(child_nodes)
        iter(child_nodes,maze, start_node, exit_node)
        # new_nodes = find_valid_child_nodes(child_nodes[0][0],child_nodes[0][1],child_nodes[0][2])

        # print(new_nodes)
        # i=1
        # while(True):

        #     # print("current child nodes")
        #     new_nodes = find_valid_child_nodes(child_nodes[0][0],child_nodes[0][1],child_nodes[0][2])
        #     # new_nodes = find_valid_child_nodes(child_nodes[0][0],child_nodes[0][1],child_nodes[0][2])
            
        #         # print(new_nodes)

        #     if (len(new_nodes)==0):
        #         # print("blocked node @ " + str(child_nodes[0][0]) + str(child_nodes[0][1]))
        #         # remove the blocked node from the list 
        #         child_nodes = child_nodes[1:]
        #     else:
        #         i= i+1
        #         for node in new_nodes:
        #             # calculate heuristic distances
        #             dist_x, dist_y = abs(exit_node[0]-node[0]), abs(exit_node[1]-node[1])  
        #             h = math.sqrt( dist_x*dist_x + dist_y*dist_y)
        #             g = child_nodes[0][2]
        #             # print(g)
        #             # input("Press Enter to continue...")
                        
        #             # e = h#+g                           #for A* algorithm

        #             e = (g_best - g) / (h+0.0001)       #for ANA*
        #             # print(exit_node[1])
        #             node.append(e)
        #             node.append(child_nodes[0][0])
        #             node.append(child_nodes[0][1])
                    
        #             if(node[0]==exit_node[0] and node[1]==exit_node[1]):
        #                 draw_canvas(canvas, maze)
        #                 root.update()
        #                 print("YEAH...GOAL FOUND!")
        #                 input("Press Enter to show current best path searched")
        #                 backtrack(parent,start_node[0],start_node[1],exit_node[0],exit_node[1])
        #                 print("Current Best solution is updated to " + str(g))
        #                 # g_goal = g
                       
        #                 print("g in loop" + str(g))
        #                 g_best = g
        #                 # print("G_value in loop" + str(current_best_g))
                        
        #                 # child_nodes = init_child_nodes
        #                 # new_nodes = []
        #                 print(child_nodes)

        #                 print(new_nodes)
                        
        #                 del child_nodes
        #                 del new_nodes
        #                 # new_nodes
        #                 maze[start_node[0]][start_node[1]] = 2
        #                 draw_canvas(canvas, maze)
        #                 root.update()
        #                 # g=0
        #                 h=0
        #                 # ana_star(maze, entrance_node, exit_node,g)
        #                 return            
        #                 # print(child_nodes)
        #                 # print(parent)
                        

                        

        #         child_nodes = new_nodes + child_nodes
                

        #         child_nodes =   sorted (child_nodes, key=getKey)
        #         child_nodes = child_nodes[::-1]                    # in case of ANA*


        #     # input("Press Enter to continue...")
        #     # draw_canvas(canvas, maze)
        #     # root.update()
            
            
        # draw_canvas(canvas, maze)
        # root.update()
    return

g_best = 50000   

def main():

    # G_value = 5000
    # while(True):
    #     # run the ana_star algorithm
    #     text = open("hard.txt")
    #     maze = [list(line.strip()) for line in text]
    #     [col, row] = np.shape(maze)
    entrance_node = (row-1, 1)
    exit_node = (0, col-2)


    #     for i in range(0, col):
    #         for j in range(0, row):
    #             # need to understand the last color part
    #             canvas.create_rectangle(j*size, i*size, (j+1)*size, (i+1)*size, fill=colors[int(maze[i][j])])

    #     canvas.pack()
    #     print("G_value entering ANA " + str(G_value))
    global g_best 
    print("Algorithm started with the intial G value of "+ str(g_best))
    prev_g=g_best
    while(1):
        # ana_star(maze, entrance_node, exit_node)
        init_time = time.time()
        ana_star(entrance_node, exit_node)
        final_time = time.time()
        print("Total time taken in this run = " + str(final_time-init_time))

        #  use if you want to stop it once best result is taken from the algorithm

        # print("updated_g = " + str( g_best))
        # if(prev_g==g_best):
        #     break
        # else:
        #     prev_g = g_best
        #     continue
    


    root.mainloop()
        


    
if __name__ == '__main__':
    main()

'''
maze co-rodinates has origin in top left corner

0-0 is - top left


to get the color value of the perticular maze, use as follows
->    int(maze[i][j])
white (0) is an unvisited node,
black(1) is a wall 
blue(2) is a visited node
yellow(3) is for start node
green(4) is for exit node
red (5) is a node on the completed path
colors = {5: "red", 4: "green", 3: "yellow", 2: "blue", 1: "black", 0: "white"}


To write a colour of the node use 
 maze[0][0] = 3
 maze[1][1] = 4

To check the status of the node 
is_valid_node(15,-15)

to check the occupancy
is_occupied_node(15,15)

    


'''