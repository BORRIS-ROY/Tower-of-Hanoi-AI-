# -*- coding: utf-8 -*-
"""
Created on Mon Dec  4 14:54:28 2023

@author: borri
"""

from array import *
import time
import os
import queue
from collections import deque
from copy import copy, deepcopy
from queue import PriorityQueue

print_rate_per_sec = 0.1
current_state = [[4, 3, 2, 1], [], [], []]  
goal_state = [[], [], [], [4, 3, 2, 1]]

class HANOI:
    def __init__(self, disks, pegs):
        self.disks = disks
        self.pegs = pegs
        self.puzzle = [[" " for _ in range(pegs)] for _ in range(disks)]

    def draw_puzzle(self):
        for i in range(self.disks):
            for j in range(self.pegs):
                print("+---", end="")
            print("+")

            for j in range(self.pegs):
                print(f"| {self.puzzle[i][j]} ", end="")
            print("|")

        for j in range(self.disks):
            print("+---", end="")
        print("+")

    def update_hanoi(self, state):
        for i in range(self.disks):
            for j in range(self.pegs):
                self.puzzle[i][j] = " "

        for i, state in enumerate(state):
            for j, disk in enumerate(state):
                self.puzzle[self.pegs - 1 - j][i] = str(disk)
                
    def display_puzzle(self):
        #os.system('cls' if os.name == 'nt' else 'clear')  # Clear the console screen
        self.draw_puzzle()
        
    def generate_neigbours(self, state):
        adjacent_states = []

        for i, peg in enumerate(state):
            if peg:
                disk = peg[-1]
                for j, other_peg in enumerate(state):
                    if i != j and (not other_peg or self.can_move(disk, other_peg[-1])):
                        new_state = [list(p) for p in state]
                        new_state[j].append(new_state[i].pop())
                        if new_state not in adjacent_states:
                            adjacent_states.append(new_state)
        return adjacent_states
    
    def is_goal_state(self,state):
        return state == [[], [], [], [4, 3, 2, 1]]
    def is_goal_state_A(self,state):
        return state == ((), (), (), (4, 3, 2, 1))
    def can_move(self, current_disk, disk_on_peg):
        return disk_on_peg is not None and current_disk < disk_on_peg
    
    def heuristic(self, state):
        goal_state = [[], [], [], [4, 3, 2, 1]]
        misplaced_disks = 0
    
        for peg_index, peg in enumerate(state):
            # Exclude the last peg
            if peg_index != 3:  
                for disk in peg:
                    #if the disk exists in the goal peg
                    if disk in goal_state[peg_index]:
                        index_in_goal = goal_state[peg_index].index(disk)
                        if index_in_goal != len(goal_state[peg_index]) - 1:
                            # Increment for each disk not in its final position
                            misplaced_disks += 1  
        # Calculate total disks in the current state
        total_disks = sum(len(peg) for peg in state) 
        # Add total disks as an additional factor
        return misplaced_disks + total_disks 

    
    def hanoi_heuristic(self, state):
    # Calculate the number of disks not yet in their final position
        goal_state = state[::-1]
        misplaced_disks = 0
    
        for peg_index, peg in enumerate(state):
            
            if peg_index != 3:
                for disk in peg:
                    if disk in goal_state[peg_index]:  
                        index_in_goal = goal_state[peg_index].index(disk)
                        if index_in_goal != len(goal_state[peg_index]) - 1:
                            misplaced_disks += 1  

        return misplaced_disks
    
    def a_star1(self, start_state):
        frontier = PriorityQueue() 
        #initial state into the frontier
        frontier.put((0, start_state))
        #dictionary to store the path
        came_from = {}  
        #dictionary to store the cost to reach each state
        cost_so_far = {start_state: 0}  
        start_time = time.time()
        traversals = 0
        start_time = time.time()
        while not frontier.empty():
            traversals +=1
            i, current_state = frontier.get()
            self.update_hanoi(current_state)
            self.display_puzzle()
            #print(current_state)
            if self.is_goal_state_A(current_state):
                end_time = time.time()
                t = ((end_time - start_time)*10)/60
                print("Goal is found")
                print("Expanded states:", traversals)
                print("Total time:", t, "minutes")
                #Return the path if goal is found
                return self.reconstruct_path(start_state, current_state, came_from)  
            for next_state in self.generate_neigbours(current_state): 
                next_state = tuple(map(tuple, next_state)) 
                new_cost = cost_so_far[current_state] + 1  
                if next_state not in cost_so_far or new_cost < cost_so_far[next_state]: 
                    cost_so_far[next_state] = new_cost  
                    priority = new_cost + self.heuristic(next_state)  
                    frontier.put((priority, next_state)) 
                    came_from[next_state] = current_state  
            time.sleep(print_rate_per_sec)
        # If no solution found, return None
        return None  
    
    def a_star2(self, start_state):
        frontier = PriorityQueue() 
        #initial state into the frontier
        frontier.put((0, start_state))
        #dictionary to store the path
        came_from = {}  
        #dictionary to store the cost to reach each state
        cost_so_far = {start_state: 0}  
        start_time = time.time()
        traversals = 0
        start_time = time.time()
        while not frontier.empty():
            traversals +=1
            i, current_state = frontier.get()
            self.update_hanoi(current_state)
            self.display_puzzle()
            #print(current_state)
            if self.is_goal_state_A(current_state):
                end_time = time.time()
                t = ((end_time - start_time)*10)/60
                print("Goal is found")
                print("Expanded states:", traversals)
                print("Total time:", t, "minutes")
                #Return the path if goal is found
                return self.reconstruct_path(start_state, current_state, came_from)  
            for next_state in self.generate_neigbours(current_state): 
                next_state = tuple(map(tuple, next_state)) 
                new_cost = cost_so_far[current_state] + 1  
                if next_state not in cost_so_far or new_cost < cost_so_far[next_state]: 
                    cost_so_far[next_state] = new_cost  
                    priority = new_cost + self.hanoi_heuristic(next_state)  
                    frontier.put((priority, next_state)) 
                    came_from[next_state] = current_state  
            time.sleep(print_rate_per_sec)
        # If no solution found, return None
        return None
    
    def reconstruct_path(self, start, goal, came_from):
        current = goal
        path = [current]
        while current != start:
            current = came_from[current]
            path.append(current)
        return path[::-1]

    
    def BFS(self, start_state, goal_state):
        #visited = set()
        visited = {}
        queue = deque([start_state])
        traversals = 0
        start_time = time.time()
        while queue:
            current_state = queue.popleft()
            traversals +=1
            self.update_hanoi(current_state)
            self.display_puzzle()
            
            if self.is_goal_state(current_state):
                end_time = time.time()
                t = ((end_time - start_time)*10)/60
                print("Visited states:", len(visited))
                print("Expanded states:", traversals)
                print("Total time:", t, "minutes")
                #print(visited)
                path = self.reconstruct_pathBFS(start_state, current_state, visited)
                return current_state, path
            
            #visited.add(tuple(map(tuple, current_state)))
            adjacent_pegs = self.generate_neigbours(current_state)
            for i in adjacent_pegs:
                it = tuple(map(tuple, i))
                if it not in visited:
                    visited[it] = tuple(map(tuple, current_state))
                #if tuple(map(tuple, i)) not in visited:
                    queue.append(i)
            time.sleep(print_rate_per_sec)
        
        print("No solution found after", traversals, "traversals")
        return False

    def reconstruct_pathBFS(self, start, goal, visited):
        current = tuple(map(tuple, goal))
        path = [current]
        start_tuple = tuple(map(tuple, start))
        while current != start_tuple:
            current = visited[current]
            path.append(current)
        return print(len(path[::-1]))

    
def main(): 
    hanoi = HANOI(disks=4, pegs=4)
    cmd = input ("Commands:\nA*(A1)\nA*Amssible(A2)\nBFS\nExit\nPlease enter the command:")
    if cmd.lower() == 'a1':
        solution = hanoi.a_star1(tuple(map(tuple, current_state)))

        if solution:
            print("Solution found:")
            for step, state in enumerate(solution):
                print(f"Step {step + 1}: {state}")
        else:
            print("No solution found.")
    elif cmd.lower() == 'a2':
        solution = hanoi.a_star2(tuple(map(tuple, current_state)))

        if solution:
            print("Solution found:")
            for step, state in enumerate(solution):
                print(f"Step {step + 1}: {state}")
        else:
            print("No solution found.")
    elif cmd.lower() == 'bfs':
        hanoi.BFS(current_state, goal_state)
    elif cmd.lower() == 'pdb':
        hanoiPDB.solve_using_a_star()
    else:
        print("Command not found!")
    


if __name__ == "__main__":
    main()


        
        