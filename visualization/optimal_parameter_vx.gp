set term postscript eps enhanced solid color font "Courier, 18"
set encoding utf8
set output "optimal_parameter_vx_theta1.eps"
set ylabel "adaptive parameters"
set xlabel "longitudinal velocity [m/s]"
set key right bottom
p "../data/output/vehicle_vx_params_1.csv" u 1:2 w l lw 2 lc "red" ti "{/Symbol q}_{11}", \
  "" u 1:3 w l lw 2 lc "blue" ti "{/Symbol q}_{12}", \
  "" u 1:4 w l lw 2 lc "dark-green" ti "{/Symbol q}_{13}"
set output

set output "optimal_parameter_vx_theta2.eps"
set ylabel "adaptive parameters"
set xlabel "longitudinal velocity [m/s]"
set key right bottom
p "../data/output/vehicle_vx_params_1.csv" u 1:5 w l lw 2 lc "red" ti "{/Symbol q}_{21}", \
  "" u 1:6 w l lw 2 lc "blue" ti "{/Symbol q}_{22}", \
  "" u 1:7 w l lw 2 lc "dark-green" ti "{/Symbol q}_{23}"

set output "optimal_parameter_vx_theta0r.eps"
set ylabel "adaptive parameters"
set xlabel "longitudinal velocity [m/s]"
set key right top
p "../data/output/vehicle_vx_params_1.csv" u 1:8 w l lw 2 lc "red" ti "{/Symbol q}_{0}", \
  "" u 1:9 w l lw 2 lc "blue" ti "k"
