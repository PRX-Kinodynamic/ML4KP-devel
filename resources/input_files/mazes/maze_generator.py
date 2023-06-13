import sys,random,os
import numpy as np

cells_x = 20
cells_y = 20
cell_size = 2

random.random()
random.seed(210896)

max_bound = 10

def random_number_gauss():
	return random.gauss(.3,.1)
def random_number_uniform(min=-max_bound,max=max_bound):
	return random.uniform(min,max)
	# return 1

def create_maze(fname,pt1=[0,0],pt2=[0,0]):
	f = open(fname,'w')
	f.write("environment:\n")
	f.write("  type: obstacle\n")
	f.write("  geometries:\n")

	create_boundaries(f)

	boxes = []

	for x in np.arange(0,cells_y*cells_x/3):
		while True:
			dx = random_number_uniform()
			dy = random_number_uniform()
			if ((dx-pt1[0])**2+(dy-pt1[1])**2 > cell_size*cell_size) and ((dx-pt2[0])**2+(dy-pt2[1])**2 > cell_size):
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

if __name__ == "__main__":
	fname = os.environ['DIRTMP_PATH']+"resources/input_files/environments/maze.yaml"
	create_maze(fname,[-9.0,-9.0],[9.0,9.0])
