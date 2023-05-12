#! /bin/zsh

get_random(){
	jot -r 1 $1 $2
}
data_dir="${DIRTMP_PATH}/data/mj_ball/floor_4x4/"
mkdir -p ${data_dir}

for i in $(seq -f %05g 0 100)
do
	duration=$(get_random 2 6.0)
	cx=$(get_random -1.00000 1.00000)
	cy=$(get_random -1.00000 1.00000)
	cz=$(get_random -1.00000 1.00000)
	plan="${data_dir}/plan_${i}.txt"
	traj="${data_dir}/traj_${i}.txt"
  echo "${duration} ${cx} ${cy} ${cz}"  > "${plan}"
	./executables/factor_graphs/mj_contact_frictions --out_traj="${traj}" --plan_file="${plan}"

done