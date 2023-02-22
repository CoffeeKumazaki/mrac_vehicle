
set term postscript eps dashed enhanced color 
set output "error_and_augumented_error.eps"
set ylabel "error [m]"
set xlabel "time [sec]"

set yr [-0.3:0.3]

p "../data/output/with_init_param/small_deadzone_0.0_vx_20_gain_custom1_straight_u04_case00_noise_0.0_e.csv"  u ($0/100):1 w l lw 3 lc "red" dt 1 ti "e", \
  "../data/output/with_init_param/small_deadzone_0.0_vx_20_gain_custom1_straight_u04_case00_noise_0.0_e1.csv" u ($0/100):1 w l lw 3 lc "blue" dt 2 ti "e_u", \
  "../data/output/with_init_param/small_deadzone_0.0_vx_20_gain_custom1_straight_u04_case00_noise_0.0_e2.csv" u ($0/100):1 w l lw 3 lc "dark-green" dt 3 ti "e_2", \
  0 lt 0 ti ""