#!/bin/zsh


# experiment="indoor"
# experiment="narrow"
experiment="warehouse"
vertices="${HOME}/Downloads/roadmaps/car_like_${experiment}/vertices.txt"
edges="${HOME}/Downloads/roadmaps/car_like_${experiment}/edges.txt"
problems="${HOME}/Downloads/roadmaps/car_like_${experiment}/problems.txt"
environment="environments/${experiment}.yaml"

plot_script="${GARY_TOOLS}/plotting/roadmaps_gaps/roadmap_with_gaps.gp"
# i=2
# i=4
i=5
# for (( i = 0; i < 6; i++ )); do
	eval executables/control/ackermann_roadmap_ctrl --problem=${i} --files/vertices=${vertices} --files/edges=${edges} --files/problems=${problems} --experiment=${experiment} --environment=${environment}

	roadmap_file="${DIRTMP_PATH}/out/roadmap.txt"
	path_file="${DIRTMP_PATH}/out/path.txt"
	plot_file=""${DIRTMP_PATH}/out/ackermann_roadmaps/${experiment}/problem_${i}.png""
	gnuplot -c ${plot_script} ${roadmap_file} ${path_file} ${plot_file}
# done