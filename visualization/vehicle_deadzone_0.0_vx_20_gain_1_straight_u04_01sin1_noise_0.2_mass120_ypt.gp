set term postscript eps solid color enhanced font "Courier, 18"
set output "vehicle_deadzone_0.2_vx_20_gain_1_straight_u04_01sin1_noise_0.2_mass120_ypt.eps"
set xlabel "Time [sec]"
set ylabel "y_e [m]"
p [0:60][-1.5:1.5] \
  "../data/output/with_init_param/vehicle_deadzone_0.2_vx_20_gain_1_straight_u04_01sin1_noise_0.2_mass120_ypt.csv" \
  every 5 u ($0*5/100):($4-$2) w l lw 2 lc "black" ti "Error", "" every 5 u ($0*5/100):4 w l lw 3 lc "blue" ti "Model", \
  "" every 5 u ($0*5/100):2 w l ti "Plant" lw 3 lc "red", 0.2 lt 0 lw 2 ti "", -0.2 lt 0 lw 2 ti ""