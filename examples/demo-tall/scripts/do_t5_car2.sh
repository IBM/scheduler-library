#!/bin/bash
./car2-scheduler-S-P3V0F0N0 -G config_files/base_me_p2.config -gS -l ML -o -s"$1" -t traces/tt02.new -f0 -R traces/norm_radar_01k_by_5_dictionary.dfn >& res_car2_tt02_"$1".out
