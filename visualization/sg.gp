set term postscript eps dash solid color enhanced font "Courier, 18"
set output "small_gain.eps"
set xr [0.01:100]
set yr [0.01:2]
set logscale x
set format x "10^{%L}"
set logscale y
set xlabel "Frequency [Hz]"
set ylabel "Magnitude"
set key left bottom

plot \
"../data/output/sg_vs5.dat"  lt 7 lw 3 w l ti "v_x =  5 m/sec", \
"../data/output/sg_vs10.dat" lc "dark-orange" dt 2 lw 3 w l ti "v_x = 10 m/sec", \
"../data/output/sg_vs20.dat" lc "dark-green" dt 5 lw 3 w l ti "v_x = 20 m/sec", \
"../data/output/sg_vs30.dat" lc "blue" dt 4 lw 3 w l ti "v_x = 30 m/sec", \
1.0 lt 0 lw 3 ti ""