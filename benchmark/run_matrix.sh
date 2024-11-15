#!/bin/bash

OUT=${1:-out}

mkdir -p $OUT

for divider in false true; do
parallel --bar --eta -j3 --shuf --header : \
   "rosrun mtc_pour mtc_pour_tams_ur5_demo _with_path_constraint:=true _spawn_objects:=true _divider:=$divider _keep_running:=false _introspection:=false __name:=mtc_{#} _workers:={workers} _solutions:={solutions} _connect_compute_attempts:={workers} > $OUT/pour_{workers}workers_{solutions}solutions_rep{repetition}_divider:$divider.log 2>&1" \
   ::: workers -1 $(seq 1 15) \
   ::: repetition $(seq 10) \
   ::: solutions 1 10 100
done
