class Node:
    def __init__(self, pos, g, h, parent=None):
        self.pos = pos
        self.g = g  # cost
        self.h = h  # heuristic
        self.f = g + h  # f(n)
        
        self.parent = parent
    
    def update(self, g, h, parent):
        self.g = g
        self.h = h
        self.f = g + h
        self.parent = parent

class PriorityQueue:
    def __init__(self):
        self.queue = []
        
    def enqueue(self, node):
        idx = 0
        while idx < len(self.queue) and self.queue[idx].f <= node.f:
            idx += 1
        self.queue.insert(idx, node)
    
    def dequeue(self):
        return self.queue.pop(0)
    
    def is_empty(self):
        return not self.queue
    
    def contains(self, position):
        return any(node.pos == position for node in self.queue)
    
    def get(self, position):
        for node in self.queue:
            if node.pos == position:
                return node
        return None

class Grid:
    def __init__(self, grid):
        self.grid = grid
        self.row = len(grid)
        self.col = len(grid[0])
    
    def is_zero(self, pos):
        return self.grid[pos[0]][pos[1]] == 0

        
def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])  # Manhattan distance

def get_neighbors(position, grid):
    neighbors = []
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # Up, Down, Left, Right
    for dx, dy in directions:
        neighbor = (position[0] + dx, position[1] + dy)
        if 0 <= neighbor[0] < grid.row and 0 <= neighbor[1] < grid.col and grid.is_zero(neighbor):
            neighbors.append(neighbor)
    return neighbors

def reconstruct_path(current):
    path = []
    while current:
        path.append(current.pos)
        current = current.parent
    return path

def bi_wa_star(start, goal, grid):
    if not grid.is_zero(start) or not grid.is_zero(goal):
        return None  # Start or goal is blocked

    open_set_start = PriorityQueue()
    open_set_goal = PriorityQueue()
    
    open_set_start.enqueue(Node(start, 0, heuristic(start, goal)))
    open_set_goal.enqueue(Node(goal, 0, heuristic(goal, start)))
    
    closed_set_start, closed_set_goal = set(), set()

    while not open_set_start.is_empty() and not open_set_goal.is_empty():
        # Search forward from start
        current_start = open_set_start.dequeue()

        if open_set_goal.contains(current_start.pos):
            goal_node = open_set_goal.get(current_start.pos)
            return reconstruct_path(current_start)[::-1] + reconstruct_path(goal_node)[1:]
        
        closed_set_start.add(current_start.pos)

        for neighbor in get_neighbors(current_start.pos, grid):
            if neighbor in closed_set_start:
                continue
            
            g_start = current_start.g + 1

            if not open_set_start.contains(neighbor):
                open_set_start.enqueue(Node(neighbor, g_start, heuristic(goal, neighbor), current_start))
            else:
                neighbor_node = open_set_start.get(neighbor)
                if g_start < neighbor_node.g:
                    neighbor_node.update(g_start, heuristic(goal, neighbor), current_start)

        # Search backward from goal
        current_goal = open_set_goal.dequeue()
        
        if open_set_start.contains(current_goal.pos):
            start_node = open_set_start.get(current_goal.pos)
            return reconstruct_path(start_node)[::-1] + reconstruct_path(current_goal)[1:]
        
        closed_set_goal.add(current_goal.pos)
        
        for neighbor in get_neighbors(current_goal.pos, grid):
            if neighbor in closed_set_goal:
                continue
            
            g_goal = current_goal.g + 1

            if not open_set_goal.contains(neighbor):
                open_set_goal.enqueue(Node(neighbor, g_goal, heuristic(start, neighbor), current_goal))
            else:
                neighbor_node = open_set_goal.get(neighbor)
                if g_goal < neighbor_node.g:
                    neighbor_node.update(g_goal, heuristic(neighbor, start), current_goal)          

    return None  # No path found

# Example usage
if __name__ == "__main__":
    grid = Grid([
        [0, 0, 0, 0, 1],
        [0, 1, 1, 1, 0],
        [0, 0, 0, 0, 0],
        [0, 1, 1, 0, 0],
        [1, 0, 0, 0, 0]
    ])

    start = (0, 3)
    goal = (4, 1)

    path = bi_wa_star(start, goal, grid)
    print("Path found:", path)
