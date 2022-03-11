set term postscript eps enhanced color
set output "./doc/image/state_no_adap.eps"

#set terminal pngcairo dashed size 1280, 960
#set output "./doc/image/state.png"

set size ratio 0.5
set yrange [60:150]
set xrange [80:260]

set xtics 20
set ytics 20
set format x ""
set format y ""
set grid
set key left top

p "data/path.txt" u 2:1 w l dashtype (20,20) lw 1 lc "black" ti "", \
  "" u 5:4 w l lt -1 ti "", "" u 7:6 w l lt -1 ti "", \
  "data/output/no_init_param/adaptive_track_vx_05_gain_0.csv" u 2:1 w l lw 4 lc "red" ti "w/o adaptation", \
#  "data/output/no_init_param/adaptive_track_vx_05_gain_10.csv" u 2:1 w l lw 4 lc "blue" ti "w/ adaptation"