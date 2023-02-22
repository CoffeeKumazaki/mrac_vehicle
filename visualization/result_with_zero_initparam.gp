set term postscript eps dashed enhanced color 
set output "result_with_zero_initparam.eps"

set xlabel "longitudinal distance [m]"
set ylabel "lateral distance [m]"
p "../data/output/no_init_param/van_deadzone_0.0_vx_20_gain_custom100_straight_u04_case00_noise_0.0.csv" w l lt -1 lw 2 ti "Plant"