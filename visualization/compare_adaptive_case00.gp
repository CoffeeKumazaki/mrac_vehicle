linewidth = 3
set style line 1 dt 1 lc "red" pt 7 ps 1.3 lw linewidth
set style line 2 dt 2 lc "red" pt 7 ps 1.3 lw linewidth
set style line 3 dt 3 lc "red" pt 7 ps 1.3 lw linewidth

set style line 4 dt 1 lc "blue" pt 5 lw linewidth ps 1.3
set style line 5 dt 2 lc "blue" pt 5 lw linewidth ps 1.3
set style line 6 dt 3 lc "blue" pt 5 lw linewidth ps 1.3

set style line 7 dt 1 lc "dark-green" pt 9 lw linewidth ps 1.8
set style line 8 dt 2 lc "dark-green" pt 9 lw linewidth ps 1.8
set style line 9 dt 3 lc "dark-green" pt 9 lw linewidth ps 1.8


set term postscript eps dashed enhanced color 

set key horizontal
set output "comparison_small_case00.eps"
set yr [-1.5: 1.5]
set ylabel "output/error [m]"
set xlabel "time [sec]"
p "../data/output/with_init_param/noadaptation_small_deadzone_0.0_vx_20_gain_custom_straight_u04_case00_noise_0.0_yr.csv" u ($0/100):1 w l lt -1 lw 3 ti "Model", \
  "../data/output/with_init_param/noadaptation_small_deadzone_0.0_vx_20_gain_custom_straight_u04_case00_noise_0.0_yp.csv" u ($0/100):1 w l ls 1 ti "FB controller", \
  "../data/output/with_init_param/small_deadzone_0.0_vx_20_gain_custom1_straight_u04_case00_noise_0.0_yp.csv" u ($0/100):1 w l ls 4 ti "MRAC", \
  "../data/output/with_init_param/noadaptation_small_deadzone_0.0_vx_20_gain_custom_straight_u04_case00_noise_0.0_e.csv" u ($0/100):1 w l ls 2 ti "", \
  "../data/output/with_init_param/small_deadzone_0.0_vx_20_gain_custom1_straight_u04_case00_noise_0.0_e.csv" u ($0/100):1 w l ls 5 ti ""


set output "comparison_vehicle_case00.eps"
set yr [-1.5: 1.5]
set ylabel "output/error [m]"
set xlabel "time [sec]"
p "../data/output/with_init_param/noadaptation_vehicle_deadzone_0.0_vx_20_gain_custom_straight_u04_case00_noise_0.0_yr.csv" u ($0/100):1 w l lt -1 lw 3 ti "Model", \
  "../data/output/with_init_param/noadaptation_vehicle_deadzone_0.0_vx_20_gain_custom_straight_u04_case00_noise_0.0_yp.csv" u ($0/100):1 w l ls 1 ti "FB controller", \
  "../data/output/with_init_param/vehicle_deadzone_0.0_vx_20_gain_custom1_straight_u04_case00_noise_0.0_yp.csv" u ($0/100):1 w l ls 4 ti "MRAC", \
  "../data/output/with_init_param/noadaptation_vehicle_deadzone_0.0_vx_20_gain_custom_straight_u04_case00_noise_0.0_e.csv" u ($0/100):1 w l ls 2 ti "", \
  "../data/output/with_init_param/vehicle_deadzone_0.0_vx_20_gain_custom1_straight_u04_case00_noise_0.0_e.csv" u ($0/100):1 w l ls 5 ti ""

set output "comparison_van_case00.eps"
set yr [-1.5: 1.5]
set ylabel "output/error [m]"
set xlabel "time [sec]"
p "../data/output/with_init_param/noadaptation_van_deadzone_0.0_vx_20_gain_custom_straight_u04_case00_noise_0.0_yr.csv" u ($0/100):1 w l lt -1 lw 3 ti "Model", \
  "../data/output/with_init_param/noadaptation_van_deadzone_0.0_vx_20_gain_custom_straight_u04_case00_noise_0.0_yp.csv" u ($0/100):1 w l ls 1 ti "FB controller", \
  "../data/output/with_init_param/van_deadzone_0.0_vx_20_gain_custom100_straight_u04_case00_noise_0.0_yp.csv" u ($0/100):1 w l ls 4 ti "MRAC", \
  "../data/output/with_init_param/noadaptation_van_deadzone_0.0_vx_20_gain_custom1_straight_u04_case00_noise_0.0_e.csv" u ($0/100):1 w l ls 2 ti "", \
  "../data/output/with_init_param/van_deadzone_0.0_vx_20_gain_custom100_straight_u04_case00_noise_0.0_e.csv" u ($0/100):1 w l ls 5 ti ""

