import math
import heapq

ROW_SIZE = None
COL_SIZE = None

# A* search algorithm by GeeksForGeeks
# Available at https://www.geeksforgeeks.org/a-search-algorithm/

# Define the Cell class
class Cell:
    def __init__(self):
        self.parent_i = 0  # Parent cell's row index
        self.parent_j = 0  # Parent cell's column index
        self.f = float("inf")  # Total cost of the cell (g + h)
        self.g = float("inf")  # Cost from start to this cell
        self.h = 0  # Heuristic cost from this cell to destination


# Check if a cell is valid (within the grid)
def is_valid(row, col, row_size, col_size):
    return (row >= 0) and (row < row_size) and (col >= 0) and (col < col_size)


# Check if a cell is unblocked
def is_unblocked(grid, row, col):
    return grid[row][col] == False


# Check if a cell is the destination
def is_destination(row, col, dest):
    return row == dest[0] and col == dest[1]


# Calculate the heuristic value of a cell (Euclidean distance to destination)
# Added functionality to path away from obstacles
def calculate_h_value(row, col, dest, grid):
    offsets = range(-3, 4)
    penalty = 0
    for x_offset in offsets:
        for y_offset in offsets:
            if not is_valid(row + x_offset, col + y_offset, ROW_SIZE, COL_SIZE): continue
            if grid[row + x_offset][col + y_offset] == True: penalty += 30 - (min(abs(x_offset), abs(y_offset)) * 2)
    return ((row - dest[0]) ** 2 + (col - dest[1]) ** 2) ** 0.5 + penalty


# Converts a full path to waypoints. A full path is not needed, only the locations where the robot stops or changes direction
def cell_details_to_waypoint_list(cell_details, dest):
    waypoints = []
    row = dest[0]
    col = dest[1]
    current_offset = (
        7,
        7,
    )  # Init current offset to be invalid to make destination a waypoint

    # Trace the path from destination to source using parent cells
    while not (
        cell_details[row][col].parent_i == row
        and cell_details[row][col].parent_j == col
    ):
        temp_row = cell_details[row][col].parent_i
        temp_col = cell_details[row][col].parent_j

        # If robot needs to make a turn, add a waypoint here
        offset = (row - temp_row, col - temp_col)
        if offset != current_offset:
            current_offset = offset
            waypoints.append((row, col))

        row = temp_row
        col = temp_col

    waypoints.reverse() # Reverse waypoints so they start from the source
    return waypoints


# Implement the A* search algorithm
def a_star_search(grid, src, dest, node):
    ROW = len(grid)
    COL = len(grid[0])
    global ROW_SIZE, COL_SIZE
    ROW_SIZE = ROW
    COL_SIZE = COL
    # node.get_logger().info(f"Size of grid: rows-{ROW} cols-{COL} valid = {(src[0] >= 0) and (src[0] < ROW) and (src[1] >= 0) and (src[1] < COL)}")
    # Check if the source and destination are valid
    if not is_valid(src[0], src[1], ROW, COL) or not is_valid(dest[0], dest[1], ROW, COL):
        node.get_logger().error(f"A*: Source or destination is invalid Sv {is_valid(src[0], src[1], ROW, COL)} Dv {is_valid(dest[0], dest[1], ROW, COL)}")
        return []

    # Check if the source and destination are unblocked
    if not is_unblocked(grid, src[0], src[1]) or not is_unblocked(
        grid, dest[0], dest[1]
    ):
        node.get_logger().error("A*: Source or the destination is blocked")
        if not is_unblocked(grid, src[0], src[1]):
            node.get_logger().info("A*: Source blocked, attempting to ignore...")

            valid_offset = False
            for offset_distance in range(1, 6):
                for x_offset in range(-offset_distance, offset_distance + 1):
                    for y_offset in range(-offset_distance, offset_distance + 1):
                        if (x_offset**2 + y_offset**2) <= offset_distance:
                            if is_unblocked(grid, src[0] + x_offset, src[1] + y_offset):
                                valid_offset = (x_offset, y_offset)
                                break
                    if valid_offset != False: break
                if valid_offset != False: break
            
            if valid_offset != False:
                node.get_logger().info(f"A*: Found valid offset {valid_offset}")
                src = (src[0] + valid_offset[0], src[1] + valid_offset[1])
            else:
                node.get_logger().info("A*: Could not find valid offset")
                return []
        else:
            return []

    # Check if we are already at the destination
    if is_destination(src[0], src[1], dest):
        node.get_logger().warn("A*: Attempted to pathfind while already at destination")
        return []

    # Initialize the closed list (visited cells)
    closed_list = [[False for _ in range(COL)] for _ in range(ROW)]
    # Initialize the details of each cell
    cell_details = [[Cell() for _ in range(COL)] for _ in range(ROW)]

    # Initialize the start cell details
    i = src[0]
    j = src[1]
    cell_details[i][j].f = 0
    cell_details[i][j].g = 0
    cell_details[i][j].h = 0
    cell_details[i][j].parent_i = i
    cell_details[i][j].parent_j = j

    # Initialize the open list (cells to be visited) with the start cell
    open_list = []
    heapq.heappush(open_list, (0.0, i, j))

    # Main loop of A* search algorithm
    while len(open_list) > 0:
        # Pop the cell with the smallest f value from the open list
        p = heapq.heappop(open_list)

        # Mark the cell as visited
        i = p[1]
        j = p[2]
        closed_list[i][j] = True

        # For each direction, check the successors
        directions = [
            (0, 1),
            (0, -1),
            (1, 0),
            (-1, 0),
            (1, 1),
            (1, -1),
            (-1, 1),
            (-1, -1),
        ]
        for dir in directions:
            new_i = i + dir[0]
            new_j = j + dir[1]

            # If the successor is valid, unblocked, and not visited
            if (
                is_valid(new_i, new_j, ROW, COL)
                and is_unblocked(grid, new_i, new_j)
                and not closed_list[new_i][new_j]
            ):
                # If the successor is the destination
                if is_destination(new_i, new_j, dest):
                    # Set the parent of the destination cell
                    cell_details[new_i][new_j].parent_i = i
                    cell_details[new_i][new_j].parent_j = j
                    # found_dest = True
                    return cell_details_to_waypoint_list(cell_details, dest)
                else:
                    # Calculate the new f, g, and h values
                    g_new = cell_details[i][j].g + 1.0
                    h_new = calculate_h_value(new_i, new_j, dest, grid)
                    f_new = g_new + h_new

                    # If the cell is not in the open list or the new f value is smaller
                    if (
                        cell_details[new_i][new_j].f == float("inf")
                        or cell_details[new_i][new_j].f > f_new
                    ):
                        # Add the cell to the open list
                        heapq.heappush(open_list, (f_new, new_i, new_j))
                        # Update the cell details
                        cell_details[new_i][new_j].f = f_new
                        cell_details[new_i][new_j].g = g_new
                        cell_details[new_i][new_j].h = h_new
                        cell_details[new_i][new_j].parent_i = i
                        cell_details[new_i][new_j].parent_j = j

    node.get_logger().info("Destination unreachable")
    return [] # No path to destination

