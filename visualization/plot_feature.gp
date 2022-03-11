# set terminal postscript eps enhanced color solid 20
# set output "feature.eps"
set terminal png size 1280, 960
set output "feature.png"

pathfile = "../data/path.txt"
ref = "../data/output/pid_vehicle.csv"
track = "../data/output/pid_track.csv"
adaptive = "../data/output/adaptive_track.csv"

track_yp = "../data/output/pid_track_yp.csv"
adaptive_yp = "../data/output/adaptive_track_yp.csv"

set multiplot
set size 0.3, 1
set origin 0, 0
set xrange [0:900]
set yrange [-100:1000]
set xtics 400
set tics scale 0

set bmargin 4

set xlabel "x [m]"
set ylabel "y [m]"

# set lmargin 0
plot pathfile using 1:2 w l lt 2 lc "gray" lw 4 title "", \
     adaptive using 1:2 w l lt 1 lc "red" lw 2 title "", \
     track using 1:2 w l lt 1 lc "blue" lw 2 title ""

reset
set xrange [0:60]

set size 0.7, 0.5
set origin 0.3, 0.0
set key right bottom
set tics scale 0.7
set yrange [-3:3]
set mytics 
set ytics 1
set xtics nomirror
set bmargin 4
set lmargin 4

set xlabel "time [sec]"
set ylabel "error [m]"

plot adaptive_yp u ($0/100):1 w l lc "red" ti "MRAC", \
     track_yp w l lc "blue" ti "PID", \
     0 lt 0 ti ""


reset
set size 0.35, 0.5
set origin 0.3, 0.5
set xrange [350:450]
set yrange [500:680]
set xtics 20
set tics scale 0

set lmargin 4
plot pathfile using 1:2 w l lt 2 lc "gray" lw 1 title "", \
     pathfile using 4:5 w l lt -1 lw 1 title "", \
     pathfile using 6:7 w l lt -1 lw 1 title "", \
     adaptive using 1:2 w l lt 1 lc "red" lw 1 title "", \
     track using 1:2 w l lt 1 lc "blue" lw 1 title ""

reset
set size 0.35, 0.5
set origin 0.65, 0.5
set xrange [350:450]
set yrange [200:300]
set xtics 50
set tics scale 0

set lmargin 4
plot pathfile using 1:2 w l lt 2 lc "gray" lw 4 title "", \
     adaptive using 1:2 w l lt 1 lc "red" lw 2 title "", \
     track using 1:2 w l lt 1 lc "blue" lw 2 title ""



unset multiplot