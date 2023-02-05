import sys,random
import os


cells_x = 20
cells_y = 20
cell_size = 2
wall = '0'
cell = ' '
unvisited = '.'

random.random()

max_bound = 10

def random_number_gauss():
	return random.gauss(.3,.1)
def random_number_uniform(min=-max_bound,max=max_bound):
	return random.uniform(min,max)
	# return 1

def create_maze(fname,start=[0,0],goal=[0,0]):
	f = open(fname,'w')
	f.write("environment:\n")
	f.write("  type: obstacle\n")
	f.write("  geometries:\n")

	#create_boundaries(f)
	map = generate_2D_mazemap(cells_y//cell_size,cells_x//cell_size)
	print_mazemap(map)
	boxes = []
	for row in range(len(map)):
		for col in range(len(map[row])):
			dx = row*cell_size - cells_x
			dy = col*cell_size - cells_y
			if map[row][col] == wall and ((dx-start[0])**2+(dy-start[1])**2 > 2) and ((dx-goal[0])**2+(dy-goal[1])**2 > 2):
				boxes.append({})
				boxes[-1]["size"] = [cell_size,cell_size]
				boxes[-1]["pos"] = [dx,dy]
	
	for i in range(len(boxes)):
		f.write("    -\n")
		f.write("      name: box_"+str(i)+"\n")
		f.write("      collision_geometry:\n")
		f.write("        type: box\n")
		f.write("        dims: ["+str(boxes[i]["size"][0])+","+str(boxes[i]["size"][1])+", .2]\n")
		f.write("        material: red\n")
		f.write("      config:\n")
		f.write("        position: ["+str(boxes[i]["pos"][0])+","+str(boxes[i]["pos"][1])+",0]\n")
		f.write("        orientation: [0,0,0,1]\n")

	f.close()

def create_random_obstacles_2D(fname,pt1=[0,0],pt2=[0,0]):
	f = open(fname,'w')
	f.write("environment:\n")
	f.write("  type: obstacle\n")
	f.write("  geometries:\n")

	#create_boundaries(f)

	boxes = []

	for x in range(0,cells_y*cells_x//3):
		while True:
			dx = random_number_uniform()
			dy = random_number_uniform()
			if ((dx-pt1[0])**2+(dy-pt1[1])**2 > 2) and ((dx-pt2[0])**2+(dy-pt2[1])**2 > 2):
				boxes.append({})
				boxes[-1]["size"] = [cell_size*random_number_gauss(),cell_size*random_number_gauss()]
				boxes[-1]["pos"] = [dx,dy]
				f.write("    -\n")
				f.write("      name: box_"+str(x)+"\n")
				f.write("      collision_geometry:\n")
				f.write("        type: box\n")
				f.write("        dims: ["+str(boxes[-1]["size"][0])+","+str(boxes[-1]["size"][1])+", .2]\n")
				f.write("        material: red\n")
				f.write("      config:\n")
				f.write("        position: ["+str(dx)+","+str(dy)+",0]\n")
				f.write("        orientation: [0,0,0,1]\n")
				break
	f.close()

def create_3D_maze(fname,pt1=[0,0],pt2=[0,0]):
	f = open(fname,'w')
	f.write("environment:\n")
	f.write("  type: obstacle\n")
	f.write("  geometries:\n")

	create_boundaries(f)

	boxes = []

	# for x in xrange(0,10):
	for x in range(0,cells_y*cells_x//15): 
		while True:
			dx = random_number_uniform()
			dy = random_number_uniform()
			dz = random_number_uniform(min=0, max=2)
			if ((dx-pt1[0])**2+(dy-pt1[1])**2 > 9) and ((dx-pt2[0])**2+(dy-pt2[1])**2 > 9):
				boxes.append({})
				boxes[-1]["size"] = [cell_size*random_number_gauss(),cell_size*random_number_gauss(),cell_size*random_number_gauss()]
				boxes[-1]["pos"] = [dx,dy,dz]
				f.write("    -\n")
				f.write("      name: box_"+str(x)+"\n")
				f.write("      collision_geometry:\n")
				f.write("        type: box\n")
				f.write("        dims: ["+str(boxes[-1]["size"][0])+","+str(boxes[-1]["size"][1])+", "+str(boxes[-1]["size"][2])+"]\n")
				f.write("        material: red\n")
				f.write("      config:\n")
				f.write("        position: ["+str(dx)+","+str(dy)+","+str(dz)+"]\n")
				f.write("        orientation: [0,0,0,1]\n")
				break
	f.close()

# Find number of surrounding cells
def surroundingCells(maze, rand_wall):
	s_cells = 0
	if (maze[rand_wall[0]-1][rand_wall[1]] == cell):
		s_cells += 1
	if (maze[rand_wall[0]+1][rand_wall[1]] == cell):
		s_cells += 1
	if (maze[rand_wall[0]][rand_wall[1]-1] == cell):
		s_cells +=1
	if (maze[rand_wall[0]][rand_wall[1]+1] == cell):
		s_cells += 1

	return s_cells

def generate_2D_mazemap(h,w):

	# add space for walls, so working area is h*w
	height = h+2
	width = w+2

	global wall
	global cell
	global unvisited
	## Main code
	# Init variables
	
	maze = [[unvisited for i in range(height)] for j in range(width)]

	# Randomize starting point and set it a cell
	starting_height = int(random.random()*height)
	starting_width = int(random.random()*width)
	if (starting_height == 0):
		starting_height += 1
	if (starting_height == height-1):
		starting_height -= 1
	if (starting_width == 0):
		starting_width += 1
	if (starting_width == width-1):
		starting_width -= 1

	# Mark it as cell and add surrounding walls to the list
	maze[starting_height][starting_width] = cell
	walls = []
	walls.append([starting_height - 1, starting_width])
	walls.append([starting_height, starting_width - 1])
	walls.append([starting_height, starting_width + 1])
	walls.append([starting_height + 1, starting_width])

	# Denote walls in maze
	maze[starting_height -1 ][starting_width] = wall
	maze[starting_height][starting_width - 1] = wall
	maze[starting_height][starting_width + 1] = wall
	maze[starting_height + 1][starting_width] = wall

	while (walls):
		# Pick a random wall
		rand_wall = walls[int(random.random()*len(walls))-1]

		# Check if it is a left wall
		if (rand_wall[1] != 0):
			if (maze[rand_wall[0]][rand_wall[1]-1] == unvisited and maze[rand_wall[0]][rand_wall[1]+1] == cell):
				# Find the number of surrounding cells
				s_cells = surroundingCells(maze, rand_wall)

				if (s_cells < 2):
					# Denote the new path
					maze[rand_wall[0]][rand_wall[1]] = cell

					# Mark the new walls
					# Upper cell
					if (rand_wall[0] != 0):
						if (maze[rand_wall[0]-1][rand_wall[1]] != cell):
							maze[rand_wall[0]-1][rand_wall[1]] = wall
						if ([rand_wall[0]-1, rand_wall[1]] not in walls):
							walls.append([rand_wall[0]-1, rand_wall[1]])


					# Bottom cell
					if (rand_wall[0] != height-1):
						if (maze[rand_wall[0]+1][rand_wall[1]] != cell):
							maze[rand_wall[0]+1][rand_wall[1]] = wall
						if ([rand_wall[0]+1, rand_wall[1]] not in walls):
							walls.append([rand_wall[0]+1, rand_wall[1]])

					# Leftmost cell
					if (rand_wall[1] != 0):	
						if (maze[rand_wall[0]][rand_wall[1]-1] != cell):
							maze[rand_wall[0]][rand_wall[1]-1] = wall
						if ([rand_wall[0], rand_wall[1]-1] not in walls):
							walls.append([rand_wall[0], rand_wall[1]-1])
				

				# Delete wall
				for i in walls:
					if (i[0] == rand_wall[0] and i[1] == rand_wall[1]):
						walls.remove(i)

				continue

		# Check if it is an upper wall
		if (rand_wall[0] != 0):
			if (maze[rand_wall[0]-1][rand_wall[1]] == unvisited and maze[rand_wall[0]+1][rand_wall[1]] == cell):

				s_cells = surroundingCells(maze, rand_wall)
				if (s_cells < 2):
					# Denote the new path
					maze[rand_wall[0]][rand_wall[1]] = cell

					# Mark the new walls
					# Upper cell
					if (rand_wall[0] != 0):
						if (maze[rand_wall[0]-1][rand_wall[1]] != cell):
							maze[rand_wall[0]-1][rand_wall[1]] = wall
						if ([rand_wall[0]-1, rand_wall[1]] not in walls):
							walls.append([rand_wall[0]-1, rand_wall[1]])

					# Leftmost cell
					if (rand_wall[1] != 0):
						if (maze[rand_wall[0]][rand_wall[1]-1] != cell):
							maze[rand_wall[0]][rand_wall[1]-1] = wall
						if ([rand_wall[0], rand_wall[1]-1] not in walls):
							walls.append([rand_wall[0], rand_wall[1]-1])

					# Rightmost cell
					if (rand_wall[1] != width-1):
						if (maze[rand_wall[0]][rand_wall[1]+1] != cell):
							maze[rand_wall[0]][rand_wall[1]+1] = wall
						if ([rand_wall[0], rand_wall[1]+1] not in walls):
							walls.append([rand_wall[0], rand_wall[1]+1])

				# Delete wall
				for i in walls:
					if (i[0] == rand_wall[0] and i[1] == rand_wall[1]):
						walls.remove(i)

				continue

		# Check the bottom wall
		if (rand_wall[0] != height-1):
			if (maze[rand_wall[0]+1][rand_wall[1]] == unvisited and maze[rand_wall[0]-1][rand_wall[1]] == cell):

				s_cells = surroundingCells(maze, rand_wall)
				if (s_cells < 2):
					# Denote the new path
					maze[rand_wall[0]][rand_wall[1]] = cell

					# Mark the new walls
					if (rand_wall[0] != height-1):
						if (maze[rand_wall[0]+1][rand_wall[1]] != cell):
							maze[rand_wall[0]+1][rand_wall[1]] = wall
						if ([rand_wall[0]+1, rand_wall[1]] not in walls):
							walls.append([rand_wall[0]+1, rand_wall[1]])
					if (rand_wall[1] != 0):
						if (maze[rand_wall[0]][rand_wall[1]-1] != cell):
							maze[rand_wall[0]][rand_wall[1]-1] = wall
						if ([rand_wall[0], rand_wall[1]-1] not in walls):
							walls.append([rand_wall[0], rand_wall[1]-1])
					if (rand_wall[1] != width-1):
						if (maze[rand_wall[0]][rand_wall[1]+1] != cell):
							maze[rand_wall[0]][rand_wall[1]+1] = wall
						if ([rand_wall[0], rand_wall[1]+1] not in walls):
							walls.append([rand_wall[0], rand_wall[1]+1])

				# Delete wall
				for i in walls:
					if (i[0] == rand_wall[0] and i[1] == rand_wall[1]):
						walls.remove(i)
				continue

		# Check the right wall
		if (rand_wall[1] != width-1):
			if (maze[rand_wall[0]][rand_wall[1]+1] == unvisited and maze[rand_wall[0]][rand_wall[1]-1] == cell):

				s_cells = surroundingCells(maze, rand_wall)
				if (s_cells < 2):
					# Denote the new path
					maze[rand_wall[0]][rand_wall[1]] = cell

					# Mark the new walls
					if (rand_wall[1] != width-1):
						if (maze[rand_wall[0]][rand_wall[1]+1] != cell):
							maze[rand_wall[0]][rand_wall[1]+1] = wall
						if ([rand_wall[0], rand_wall[1]+1] not in walls):
							walls.append([rand_wall[0], rand_wall[1]+1])
					if (rand_wall[0] != height-1):
						if (maze[rand_wall[0]+1][rand_wall[1]] != cell):
							maze[rand_wall[0]+1][rand_wall[1]] = wall
						if ([rand_wall[0]+1, rand_wall[1]] not in walls):
							walls.append([rand_wall[0]+1, rand_wall[1]])
					if (rand_wall[0] != 0):	
						if (maze[rand_wall[0]-1][rand_wall[1]] != cell):
							maze[rand_wall[0]-1][rand_wall[1]] = wall
						if ([rand_wall[0]-1, rand_wall[1]] not in walls):
							walls.append([rand_wall[0]-1, rand_wall[1]])

				# Delete wall
				for i in walls:
					if (i[0] == rand_wall[0] and i[1] == rand_wall[1]):
						walls.remove(i)

				continue

		# Delete the wall from the list anyway
		for i in walls:
			if (i[0] == rand_wall[0] and i[1] == rand_wall[1]):
				walls.remove(i)
		


	# Mark the remaining unvisited cells as walls
	for i in range(0, height):
		for j in range(0, width):
			if (maze[i][j] == unvisited):
				maze[i][j] = wall

	# Print final maze
	return maze

def print_mazemap(map):
	for i in map:
		for j in i:
			print(j,end="")
		print()

def create_boundaries(f):
	f.write("    -\n")
	f.write("      name: upper\n")
	f.write("      collision_geometry:\n")
	f.write("        type: box\n")
	f.write("        dims: [21,.5,.2]\n")
	f.write("        material: red\n")
	f.write("      config:\n")
	f.write("        position: [0,10.25,0]\n")
	f.write("        orientation: [0,0,0,1]\n")

	f.write("    -\n")
	f.write("      name: lower\n")
	f.write("      collision_geometry:\n")
	f.write("        type: box\n")
	f.write("        dims: [21,.5,.2]\n")
	f.write("        material: red\n")
	f.write("      config:\n")
	f.write("        position: [0,-10.25,0]\n")
	f.write("        orientation: [0,0,0,1]\n")

	f.write("    -\n")
	f.write("      name: left\n")
	f.write("      collision_geometry:\n")
	f.write("        type: box\n")
	f.write("        dims: [.5,21,.2]\n")
	f.write("        material: red\n")
	f.write("      config:\n")
	f.write("        position: [-10.25,0,0]\n")
	f.write("        orientation: [0,0,0,1]\n")

	f.write("    -\n")
	f.write("      name: right\n")
	f.write("      collision_geometry:\n")
	f.write("        type: box\n")
	f.write("        dims: [.5,21,.2]\n")
	f.write("        material: red\n")
	f.write("      config:\n")
	f.write("        position: [10.25,0,0]\n")
	f.write("        orientation: [0,0,0,1]\n")
	'''
	f.write("    -\n")
	f.write("      name: upper\n")
	f.write("      collision_geometry:\n")
	f.write("        type: box\n")
	f.write("        dims: [41,.5,.2]\n")
	f.write("        material: red\n")
	f.write("      config:\n")
	f.write("        position: [0,20.25,0]\n")
	f.write("        orientation: [0,0,0,1]\n")

	f.write("    -\n")
	f.write("      name: lower\n")
	f.write("      collision_geometry:\n")
	f.write("        type: box\n")
	f.write("        dims: [41,.5,.2]\n")
	f.write("        material: red\n")
	f.write("      config:\n")
	f.write("        position: [0,-20.25,0]\n")
	f.write("        orientation: [0,0,0,1]\n")

	f.write("    -\n")
	f.write("      name: left\n")
	f.write("      collision_geometry:\n")
	f.write("        type: box\n")
	f.write("        dims: [.5,41,.2]\n")
	f.write("        material: red\n")
	f.write("      config:\n")
	f.write("        position: [-20.25,0,0]\n")
	f.write("        orientation: [0,0,0,1]\n")

	f.write("    -\n")
	f.write("      name: right\n")
	f.write("      collision_geometry:\n")
	f.write("        type: box\n")
	f.write("        dims: [.5,41,.2]\n")
	f.write("        material: red\n")
	f.write("      config:\n")
	f.write("        position: [20.25,0,0]\n")
	f.write("        orientation: [0,0,0,1]\n")
	'''

if __name__ == "__main__":
	
	script_dir = os.path.dirname(__file__)
	fname = os.path.join(script_dir,"maze.yaml")
	create_maze(fname,[-9.5,-9.5],[9.5,9.5])
