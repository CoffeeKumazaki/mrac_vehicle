linewidth = 3

set style line 1 dt 1 lc "red" pt 7 ps 1.3 lw linewidth
set style line 2 dt 2 lc "red" pt 7 ps 1.3 lw linewidth
set style line 3 dt 3 lc "red" pt 7 ps 1.3 lw linewidth

set style line 4 dt 1 lc "blue" pt 5 lw linewidth ps 1.3
set style line 5 dt 2 lc "blue" pt 5 lw linewidth ps 1.3
set style line 6 dt 3 lc "blue" pt 5 lw linewidth ps 1.3

set style line 7 dt 1 lc "dark-green" pt 9 lw linewidth ps 1.3
set style line 8 dt 2 lc "dark-green" pt 9 lw linewidth ps 1.3
set style line 9 dt 3 lc "dark-green" pt 9 lw linewidth ps 1.3

height = 0.29
lms  = 0.11
rms  = 0.97
hms  = 0.01

bms3 = 0.09
tms3 = bms3 + height
bms2 = tms3 + hms
tms2 = bms2 + height
bms1 = tms2 + hms
tms1 = bms1 + height

set term postscript eps dashed enhanced color 
set output "../adaptive_param_l0.eps"

set multiplot

set key horizontal
set logscale x
set key left top

set xr [0.1:30]

set lmargin screen lms
set rmargin screen rms
set tmargin screen tms3
set bmargin screen bms3

set xlabel ""
set key left bottom
set ylabel "{/Symbol q}_{3}"
set ytics -20, 5
set yr[-18:2]
set xlabel "{/Symbol L}_0"

p "../../data/output/small_vx_20_params.csv" u 1:8 w lp ls 1 ti "" \
, "../../data/output/van_vx_20_params.csv" u 1:8 w lp ls 4 ti "" \
, "../../data/output/vehicle_vx_20_params.csv" u 1:8 w lp ls 7 ti "" \

set ylabel "{/Symbol q}_{1}"
set key left top
set lmargin screen lms
set rmargin screen rms
set tmargin screen tms1
set bmargin screen bms1
set format x ""
unset logscale y
set yr[-1000: 4000]
set ytics 0, 1000
set xlabel ""

p "../../data/output/small_vx_20_params.csv" u 1:2 w lp ls 1 ti "Vehicle 1" \
, 10000 w l dt 1 lc "black" lw linewidth ti "{/Symbol q}_{i0}" \
,  "" u 1:3 w lp ls 2 ti "" \
,  "" u 1:4 w lp ls 3 ti "" \
, "../../data/output/van_vx_20_params.csv" u 1:2 w lp ls 4 ti "Vehicle 2" \
, 10000 w l dt 2 lc "black" lw linewidth ti "{/Symbol q}_{i1}" \
,  "" u 1:3 w lp ls 5 ti "" \
,  "" u 1:4 w lp ls 6 ti "" \
, "../../data/output/vehicle_vx_20_params.csv" u 1:2 w lp ls 7 ti "Vehicle 3" \
,  "" u 1:3 w lp ls 8 ti "" \
,  "" u 1:4 w lp ls 9 ti "" \
, 10000 w l dt 3 lc "black" lw linewidth ti "{/Symbol q}_{i2}" 


set lmargin screen lms
set rmargin screen rms
set tmargin screen tms2
set bmargin screen bms2

set ylabel "{/Symbol q}_{2}"
set logscale y
set format y "10^{%L}"
set yr[2: 200000]
set ytics (1, 10, 100, 1000, 10000, 100000)
p "../../data/output/small_vx_20_params.csv" u 1:5 w lp ls 1 ti "" \
,  "" u 1:6 w lp ls 2 ti "" \
,  "" u 1:7 w lp ls 3 ti "" \
, "../../data/output/van_vx_20_params.csv" u 1:5 w lp ls 4 ti "" \
,  "" u 1:6 w lp ls 5 ti "" \
,  "" u 1:7 w lp ls 6 ti "" \
, "../../data/output/vehicle_vx_20_params.csv" u 1:5 w lp ls 7 ti "" \
,  "" u 1:6 w lp ls 8 ti "" \
,  "" u 1:7 w lp ls 9 ti ""


unset multiplot
set output

if (0) {
set key horizontal
set logscale x
set key left top
set xr [0.1:30]
set ylabel "{/Symbol q}_{1}"
p "../../data/output/small_vx_20_params.csv" u 1:2 w lp ls 1 ti "Vehicle 1" \
,  "" u 1:3 w lp ls 2 ti "" \
,  "" u 1:4 w lp ls 3 ti "" \
, "../../data/output/van_vx_20_params.csv" u 1:2 w lp ls 4 ti "Vehicle 2" \
,  "" u 1:3 w lp ls 5 ti "" \
,  "" u 1:4 w lp ls 6 ti "" \
, "../../data/output/vehicle_vx_20_params.csv" u 1:2 w lp ls 7 ti "Vehicle 3" \
,  "" u 1:3 w lp ls 8 ti "" \
,  "" u 1:4 w lp ls 9 ti "" 

set output 

set output "../adaptive_param_l0_theta3.eps"
set key left bottom
set ylabel "{/Symbol q}_{3}"

p "../../data/output/small_vx_20_params.csv" u 1:8 w lp ls 1 ti "Vehicle 1" \
, "../../data/output/van_vx_20_params.csv" u 1:8 w lp ls 4 ti "Vehicle 2" \
, "../../data/output/vehicle_vx_20_params.csv" u 1:8 w lp ls 7 ti "Vehicle 3"
set output 



set key left top
set output "../adaptive_param_l0_theta2.eps"
set ylabel "{/Symbol q}_{2}"
set logscale y
p "../../data/output/small_vx_20_params.csv" u 1:5 w lp ls 1 ti "Vehicle 1" \
,  "" u 1:6 w lp ls 2 ti "" \
,  "" u 1:7 w lp ls 3 ti "" \
, "../../data/output/van_vx_20_params.csv" u 1:5 w lp ls 4 ti "Vehicle 2" \
,  "" u 1:6 w lp ls 5 ti "" \
,  "" u 1:7 w lp ls 6 ti "" \
, "../../data/output/vehicle_vx_20_params.csv" u 1:5 w lp ls 7 ti "Vehicle 3" \
,  "" u 1:6 w lp ls 8 ti "" \
,  "" u 1:7 w lp ls 9 ti ""

}