#! /bin/zsh

get_random(){
	jot -r 1 $1 $2
}
# experiment_id=0
# experiment_id="2" # Friction="1 0 X"
# experiment_id="3" # Friction="1 X 0"
# experiment_id="4" # Friction="1 X X"
# experiment_id="5" # Friction="1 0 0"
data_dir="${DIRTMP_PATH}/data/mushr/midpatch/"
mkdir -p ${data_dir}/${experiment_id}/{plans,trajs,observation}

rm -f ${data_dir}/${experiment_id}/plans/*
rm -f ${data_dir}/${experiment_id}/trajs/*
rm -f ${data_dir}/${experiment_id}/observation/*

for i in $(seq -f %05g 0 100)
do
	duration=$(get_random 5.0 25.0)
	steer=$(get_random -0.2000 0.2000)
	throttle=$(get_random 0.0000 1.0000)
	if [[ "$experiment_id" == "0" ]]; then
		x="0.0"
		y="0.0"
	elif [[ "$experiment_id" == "1" ]]; then
		x=$(get_random -1.50 2.00)
		y=$(get_random -5.00 5.00)
	elif [[ "$experiment_id" == "2" ]]; then
		steer="0.0"
		throttle="1.0"
		duration="10"
		x="0.0"
		y=$(get_random -5.0000 5.0000)
	elif [[ "$experiment_id" == "3" ]]; then
		steer="0.0"
		throttle="1.0"
		duration="10"
		x="0.0"
		y=$(get_random -5.0000 5.0000)
	elif [[ "$experiment_id" == "4" ]]; then
		steer="0.0"
		throttle="1.0"
		duration="10"
		x="0.0"
		y=$(get_random -5.0000 5.0000)
	elif [[ "$experiment_id" == "5" ]]; then
		steer="0.0"
		throttle="1.0"
		duration="10"
		x="0.0"
		y=$(get_random -5.0000 5.0000)
	fi
	z="0.025"
	plan="${data_dir}/${experiment_id}/plans/plan_${i}.txt"
	traj="${data_dir}/${experiment_id}/trajs/traj_${i}.txt"
	obs="${data_dir}/${experiment_id}/observation/obs_${i}.txt"
  echo "${duration} ${steer} ${throttle}"  > "${plan}"
	./executables/factor_graphs/mj_contact_frictions \
			--plan_file=${plan} --out_traj=${traj} \
			--plant/start_state/xyz="[$x, $y, $z]" \
			--observation/filename="${obs}"

done