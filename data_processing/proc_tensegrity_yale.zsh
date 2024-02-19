#!/bin/zsh

data_file="${DIRTMP_PATH}/data/tensegrity/processed_data/"
executable="${DIRTMP_PATH}/bin/executables/factor_graphs/tensegrity_rod_estimation"
json_to_txt="${DIRTMP_PATH}/data_processing/tensegrity_positions_to_txt.py"
python_cmd=$(which python3)
gnuplot_cmd="${GARY_TOOLS}/plotting/tensegrity/smoothing_yale.gp"
# time pos rod_01_end_pt1 rod_01_end_pt2 
# rod_23_end_pt1 rod_23_end_pt2 
# rod_45_end_pt1 rod_45_end_pt2 

# x_idx (1 2 3), (4 5 6), (7 8 9), (10 11 12), (13 14 15), (16 17 18) 
# /Users/Gary/pracsys/ML4KP-devel/data/tensegrity/processed_data/R2S2Rccw_1/processed_data.json
# 1 2 3 4 5 6
# 1   3   5
# 1   2   3
rods0=("01_1" "23_1" "45_1" )
rods1=("01_2" "23_2" "45_2" )
for dir in $(ls ${data_file}); do 
	curr_dir="${data_file}/${dir}"
	eval "${python_cmd} ${json_to_txt} -o ${curr_dir}/positions.txt -j ${curr_dir}/processed_data.json"
	input_file="--/input/file=${curr_dir}/positions.txt"
	t_idx=0
	for (( i = 1; i < 4; i++ )); do
		rod0=$rods0[i]
		rod1=$rods1[i]
		out_file="--out/file0=${curr_dir}/estimation.txt"
		let "x0_idx = (i-1) * 6 + 1 "
		let "x1_idx = x0_idx + 3 "
		idxs="--input/t_idx=${t_idx} --input/x0_idx=${x0_idx} --input/x1_idx=${x1_idx}"
		# echo ${x_idx}
		# echo ${executable} "${input_file} ${out_file} ${idxs}"
		eval "${executable} ${input_file} ${out_file} ${idxs}"

		awk_command0="{if(\$12==0)print>\"${curr_dir}/estimation_${rod0}.txt\"}"
		awk_command1="{if(\$12==1)print>\"${curr_dir}/estimation_${rod1}.txt\"}"
		awk ${awk_command0} "${curr_dir}/estimation.txt"
		awk ${awk_command1} "${curr_dir}/estimation.txt"
	done
	eval "gnuplot -c ${gnuplot_cmd} ${dir}"
	# break
	# echo $dir
done

