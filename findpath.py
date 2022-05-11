# Path Finding uisng A* Algorithm

# Import modules
import pygame
import math
from queue import PriorityQueue

# Constants for the window
WIDTH = 650
WIN = pygame.display.set_mode((WIDTH, WIDTH))
pygame.display.set_caption("Path Finding Using A* Algorithm")

# Constants for the colors of the game
RED = (255, 0, 0)			# Nodes already visited
GREEN = (0, 255, 0)			# Neighbor nodes
BLUE = (0, 0, 255)			# Path
BLACK = (0, 0, 0)			# Obstacles
WHITE = (255, 255, 255)		# Nodes that could be visited
GREY = (128, 128, 128)		# Grid
ORANGE = (255, 165, 0)		# Start node	
PURPLE = (128, 0, 128)		# End node	

# Node class
class Node:
	"""
	Class that keep track of the different nodes
	"""
	def __init__(self, row, col, width, total_rows):
		# Defining attributes
		self.row = row
		self.col = col
		self.x = row * width
		self.y = col * width
		self.color = WHITE
		self.neighbors = []
		self.width = width
		self.total_rows = total_rows

	def get_pos(self):
		"""
		Function that returns the position for every node.
		"""
		return self.row, self.col

	def is_obstacle(self):
		"""
		Function that returns a True if the node that we're looking at is an obstacle.
		False otherwise.
		"""
		return self.color == BLACK


	# Functions that set the color of the node depending of the functions above
	def reset(self):
		"""
		Function that sets the node-color to white
		"""
		self.color = WHITE

	def make_closed(self):
		"""
		Function that sets the node-color to red if the function 'is_closed()' is True.
		"""
		self.color = RED

	def make_open(self):
		"""
		Function that sets the node-color to green if the function 'is_open()' is True.
		"""
		self.color = GREEN

	def make_obstacle(self):
		"""
		Function that sets the node-color to black if the function 'is_obstacle()' is True.
		"""
		self.color = BLACK

	def make_start(self):
		"""
		Function that sets the node-color to orange if the function 'is_start()' is True.
		"""
		self.color = ORANGE

	def make_end(self):
		"""
		Function that sets the node-color to brown if the function 'is_end()' is True.
		"""
		self.color = PURPLE

	def make_path(self, current, start):
		"""
		Function that sets the node-color to blue with the shortest path of the closed nodes.
		"""
		if current == start:
			self.color = ORANGE
		else:
			self.color = BLUE

	def draw(self, win):
		"""
		Function that draws a cube (node) while reaserching for the path to the end-node.
		"""
		pygame.draw.rect(win, self.color, (self.x, self.y, self.width, self.width))

	def update_neighbors(self, grid):
		"""
		Function that takes the grid as an argument and determines all the neighbors of a node.
		"""
		self.neighbors = []

		# Conditions for the neighbors
		if self.row < self.total_rows - 1 and not grid[self.row + 1][self.col].is_obstacle():
			# A neighbor could be placed down the current node
			self.neighbors.append(grid[self.row + 1][self.col])

		if self.row > 0 and not grid[self.row - 1][self.col].is_obstacle():
			# A neighbor could be placed up the current node
			self.neighbors.append(grid[self.row - 1][self.col])

		if self.col < self.total_rows - 1 and not grid[self.row][self.col + 1].is_obstacle():
			# A neighbor could be placed right the current node
			self.neighbors.append(grid[self.row][self.col + 1])

		if self.col > 0 and not grid[self.row][self.col - 1].is_obstacle():
			# A neighbor could be placed left the current node
			self.neighbors.append(grid[self.row][self.col - 1])
	
	def __lt__(self, other):
		"""
		Function that handles the comparisson for two nodes.
		"""
		return False


# Reconstructing the shortest path
def reconstructed_path(parent, start, current, draw):
	"""
	Function that reconstruct the path taking the end node (current) and the parent dictionary backwards.
	"""
	while current in parent:
		current = parent[current]
		current.make_path(current, start)
		draw()	


# Heuristic function
def heurisitc(p1, p2):
	"""
	Function that determines H(n) through Manhattan distance
	n1, n2 --> points (x, y)
	returns: the L distance (Manhattan distance) between those two points as an integer
	"""

	# Coordinates
	x1, y1 = p1
	x2, y2 = p2

	return abs(x2 - x1) + abs(y2 - y1) #"Euclide distance"


# A* algorithm
def algorithm(draw, grid, start, end):
	"""
	Function that handles the A* algorithm.
	returns: False if there isn't a way to go to the end node.
	"""

	count = 0
	open_set = PriorityQueue()
	open_set.put((0, count, start))	# Start node in the open set
	parent = {}						# Dictionary that tell us where the nodes came from

	g_score = {node: float("inf") for row in grid for node in row}	# Current shortest distance to get from the start node to the current node
	g_score[start] = 0												# g(0) = 0
	f_score = {node: float("inf") for row in grid for node in row}	# Current predicted distance from the current node to the end node
	f_score[start] = heurisitc(start.get_pos(), end.get_pos())		# f(n) = g(n) + h(n). At the start g(0) = 0, --> f(0) = h(0) 

	open_set_hash = {start}			# Keep track of the open set

	while not open_set.empty():
		# There are nodes to check out
		for event in pygame.event.get():
			if event.type == pygame.QUIT:
				pygame.quit()

		current = open_set.get()[2]		# Current node we're looking at
		open_set_hash.remove(current)	# Avoiding duplicated nodes

		if current == end:
			# Making the path
			reconstructed_path(parent, start, end, draw)
			end.make_end()
			return True

		for neighbor in current.neighbors:
			# Adding 1 from node to node
			temp_g_score = g_score[current] + 1

			if temp_g_score < g_score[neighbor]:
				# Updating the new value fro the path
				parent[neighbor] = current
				g_score[neighbor] = temp_g_score
				f_score[neighbor] = temp_g_score + heurisitc(neighbor.get_pos(), end.get_pos())

				if neighbor not in open_set_hash:
					# Neighbor not in the open set
					count += 1
					open_set.put((f_score[neighbor], count, neighbor))
					open_set_hash.add(neighbor)
					neighbor.make_open()

		draw()

		if current != start:
			# The node that we checked out have already been considered, so it goes to the close set
			current.make_closed()

	return False


def make_grid(rows, width):
	"""
	Function that takes rows and width as parameter and determine the grid of the game.
	returns: the grid, wich is a list of lists.
	"""
	grid = []
	gap = width // rows 		# Width for each cube

	for i in range(rows):		# i --> row
		grid.append([])	
		for j in range(rows):	# j --> col
			node = Node(i, j, gap, rows)
			grid[i].append(node)

	return grid


def draw_grid(win, rows, width):
	"""
	Function that takes win, rows and width as parameters and draw the grid
	in order to visualize it better in the game.
	"""

	gap = width // rows

	for i in range(rows):
		pygame.draw.line(win, GREY, (0, i*gap), (width, i*gap))
		for j in range(rows):
			pygame.draw.line(win, GREY, (j*gap, 0), (j*gap, width))


def draw(win, grid, rows, width):
	"""
	Function that draws everything in the game: the nodes (cubes) and the grid.
	"""

	win.fill(WHITE)				# Everything in white color every frame

	for row in grid:
		for node in row:
			node.draw(win)		# Drawing all the nodes with their corresponded color

	draw_grid(win, rows, width)	# Drawing the grid on top the nodes
	pygame.display.update()


def get_mouse_event(pos, rows, width):
	"""
	Function that takes the position of the mouse on the screen and width the gap between each
	node, calculate its row and col posiiton.
	returns: row, col posiiton
	"""
	gap = width // rows
	x, y = pos

	row = x // gap
	col = y // gap

	return row, col


# Main function
def main(win, width):
	"""
	Function that creates the hole game with the helper function nad the Node class.
	"""

	# Making the grid
	ROWS = 50
	grid = make_grid(ROWS, width)

	# Start and end position
	start = None
	end = None

	# Boolean variable
	run = True

	while run:
		draw(win, grid, ROWS, width)

		# For every event that occurs
		for event in pygame.event.get():
			if event.type == pygame.QUIT:
				run = False


			if pygame.mouse.get_pressed()[0]:
				# Left mouse button pressed
				pos = pygame.mouse.get_pos()
				row, col = get_mouse_event(pos, ROWS, width)
				node = grid[row][col]	# Node corresponded to the mouse event 

				if not start and node != end:
					# Making the start node
					start = node
					start.make_start()

				elif not end and node != start:
					# Making the end node
					end = node
					end.make_end()

				elif node != start and node != end:
					# Making the obsctacles
					node.make_obstacle()


			elif pygame.mouse.get_pressed()[2]:
				# Right mouse button pressed
				pos = pygame.mouse.get_pos()
				row, col = get_mouse_event(pos, ROWS, width)
				node = grid[row][col]	# Node corresponded to the mouse event 
				node.reset()

				if node == start:
					start = None
				elif node == end:
					end = None


			if event.type == pygame.KEYDOWN:
				if event.key == pygame.K_SPACE and start and end:
					# Starting the algorithm
					for row in grid:
						for node in row:
							node.update_neighbors(grid)

					algorithm(lambda: draw(win, grid, ROWS, width), grid, start, end)

				if event.key == pygame.K_c:
					# 'c' key was pressed to clear the scren
					start = None
					end = None
					grid = make_grid(ROWS, width)

	pygame.quit()



if __name__ == "__main__":
    main(WIN, WIDTH)