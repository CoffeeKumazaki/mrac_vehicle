set terminal pngcairo dashed size 1280, 960
set output "./doc/image/ye.png"

set term postscript eps enhanced color
set output "./doc/image/ye.eps"

set xlabel "time [sec]"
set ylabel "error [m]"

set key left top

p "data/output/no_init_param/adaptive_track_vx_05_gain_0_yp.csv" u ($0/100):1 w l lw 4 lc "red" ti "w/o adaptation", \
  "data/output/no_init_param/adaptive_track_vx_05_gain_10_yp.csv" u ($0/100):1 w l lw 4 lc "blue" ti "w/ adaptation", \
  2-0.45 lc "gray" lw 2 ti "", \
  -2+0.45 lc "gray" lw 2 ti ""