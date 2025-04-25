'''You will write a program to solve planning problems in Vacuum World, a grid-based en-
vironment in which some cells are dirty. Your robot must return a plan—a sequence of
actions—that moves it around the grid and vacuums all dirty cells. The robot can move in
the four cardinal directions (North, South, East, West) and can vacuum when on a dirty
cell. All actions have uniform cost.
You must implement two search algorithms:
• Uniform-Cost Search (UCS)
• Depth-First Search (DFS)
You may use any programming language, but you may only use standard libraries.
Do not import or include any third-party packages.'''
import sys
import heapq
from dataclasses import *

'''DFS: avoid infinite loops or cycles. '''

'''STATE CLASS:
    position: (x,y)
    dirty_cells - set of cells to be cleaned
    NEIGHBORS - Directions the position can go'''
@dataclass
class State:
    position: tuple
    dirty_cells: set = field(default_factory=set)
    NEIGHBORS = { #N,S,E,W moves
        'N': (-1, 0),
        'S': (1, 0),
        'E': (0, 1),
        'W': (0, -1)
    }

    def __eq__(self, other):
        return self.position == other.position and self.dirty_cells == other.dirty_cells

    def __hash__(self): #Allow State Class Objects to be sets
        return hash((self.position, tuple(sorted(self.dirty_cells))))

'''Different moves as single letters
N - north
S - South
E - east
W - west
V - vacuum'''
MOVES = ['N', 'S', 'E', 'W', 'V']

'''Reads provided file by main function and terminal command'''
def read_file(file):
    f = open(file, 'r', encoding='utf-16') #open file in read mode - encoded in utf-16
    cols = int(f.readline()) #placeholder to read rows
    rows = int(f.readline()) #amt of rows in map

    grid = []
    dirty_cell_set = set() #store position of dirty cells from grid
    blocked_cell_set = set()
    start = None #placeholder for starting point

    for row in range(rows):
        curr_line = f.readline().strip() #read current line without extra spaces
        #print(curr_line) #debugging and visually see grid you are working with
        curr_row = []
        for col in range(len(curr_line)):
            char = curr_line[col] #current element
            curr_row.append(char)
            if char == '*': #if dirty cell, add to dirty cell set to use later
                dirty_cell_set.add((row, col))
            if char == '@': #starting position
                start = (row, col)
            if char == '#':
                blocked_cell_set.add((row,col))
        grid.append(curr_row) #create grid with starting position

    return grid, State(start, dirty_cell_set), blocked_cell_set #return grid with starting position


'''Depth First Search algorithm - move through all nodes - not looking for best path just going through all nodes'''
def DFS(grid, start, blocked_cells):
    stack = [(start, [])] #stack of moves and path
    visited = set()
    nodes_gen = 1 #generated nodes
    nodes_exp = 0 #expanded nodes

    while stack:
        Spot, path = stack.pop() #last inserted spot
        if Spot in visited:
            continue
        visited.add(Spot) #if spot isnt already visited, add to visited and add 1 to nodes
        nodes_exp += 1
        if len(Spot.dirty_cells) == 0:
            return path, nodes_gen, nodes_exp #if finished cleaning, end and return
        x, y = Spot.position
        for move in MOVES:
            next = (x,y)
            new_dirty = set(Spot.dirty_cells)
            if move == 'V': #if vacuum is the next move, and  clean cell and remove it from new dirty set
                if(x,y) in new_dirty:
                    new_dirty.remove((x,y))
                next_place = State(next, new_dirty) #start going to the next dirty square
            else:
                r, c = State.NEIGHBORS[move] #move in direction of active move direction (N,S,W,E)
                nextx, nexty = x + r, y + c #actually change the position by coords based on moves
                #make sure position is within the grid, and not about to move into a blocked cell
                if 0 <= nextx < len(grid) and 0 <= nexty < len(grid[0]) and grid[nextx][nexty] not in blocked_cells:
                    next = (nextx, nexty)
                    next_place = State(next, new_dirty)
                else:
                    continue
            if next_place not in visited: #if the next place is not part of visited, move there and add to path
                new_path = path + [move]
                stack.append((next_place, new_path))
                nodes_gen += 1 #add 1 to generated nodes
    return [], nodes_gen, nodes_exp #return path, generated nodes, expanded nodes





'''look for most cost effective path'''
def UCS(grid, start, blocked_cells):
    queue = [] #nodes to be explored, and cost to get there
    tie_breaker_count = 0 #break ties in queue
    heapq.heappush(queue, (0, tie_breaker_count, start, []))
    visited = set()
    nodes_gen = 1
    nodes_exp = 0

    while queue:
        cost, _, Spot, path = heapq.heappop(queue) #get cost, position, and path of certain node in grid

        if Spot in visited:
            continue

        visited.add(Spot)
        nodes_exp += 1

        if len(Spot.dirty_cells) == 0: #if finished cleaning return path, generated nodes, and explored nodes
            return path, nodes_gen, nodes_exp

        x, y = Spot.position #current position
        for move in MOVES: #cycle through each move
            next = (x, y) #placeholder until later for the next cell on grid
            new_dirty = set(Spot.dirty_cells)

            if move == 'V':
                if (x, y) in new_dirty: #if vacuum and next node is dirty, remove from new_dirty and vacuum it
                    new_dirty.remove((x, y))
                next_place = State(next, new_dirty) #next goal is next dirty spot
            else:
                r, c = State.NEIGHBORS[move] #next position temp
                nextx, nexty = x + r, y + c #next position on grid
                #check if next position is within boundaries and not blocked
                if 0 <= nextx < len(grid) and 0 <= nexty < len(grid[0]) and grid[nextx][nexty] not in blocked_cells:
                    next = (nextx, nexty) #next position
                    next_place = State(next, new_dirty) #go to next position
                else:
                    continue

            if next_place not in visited:
                new_path = path + [move] #add next place to path
                tie_breaker_count += 1  # add one in case of tie
                heapq.heappush(queue, (cost + 1, tie_breaker_count, next_place, new_path))
                nodes_gen += 1

    return [], nodes_gen, nodes_exp  # end case - impossible



def main():
    #print("running") #debug
    if len(sys.argv) != 3:
        print("Usage: python3 planner.py <DFS/UCS call> <world file>")
        sys.exit(1)
    #print("reading") #debug
    grid, start, blocked_cells = read_file(sys.argv[2])
    if(sys.argv[1] == "uniform-cost"):
        action, gen, exp = UCS(grid, start, blocked_cells)
    elif(sys.argv[1] == "depth-first"):
        action, gen, exp = DFS(grid, start, blocked_cells)
    else:
        print("Enter either depth-first or uniform-cost as the first argument after planner.py")
    for moves in action:
        print(moves)
    print("Nodes Generated: ", gen)
    print("Nodes Expanded: ", exp)


if __name__ == "__main__":
    main()
