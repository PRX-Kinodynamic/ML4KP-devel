#!/bin/bash


cuttoff=1
gain=1
gain_goal=1
executable="${DIRTMP_PATH}bin/executables/factor_graphs/SE3_potential_field --repulsive --normalize --cuttoff=${cuttoff} --gain_goal=${gain_goal} --gain=${gain}"

tmp_file="/tmp/pf"
se3_file="/tmp/se3"
comb_file="/tmp/comb"

step=0.2
rm ${tmp_file}
rm ${se3_file}
rm ${comb_file}
for x in $(seq -10 ${step} 10)
do
	for z in $(seq 0 ${step} 70)
	do
  	echo "0.8775826 0 0 0.4794255 ${x} 0 ${z}" >> ${tmp_file}
  done
done

cat ${tmp_file} | ${executable} > ${se3_file}
paste ${tmp_file} ${se3_file} > /tmp/comb