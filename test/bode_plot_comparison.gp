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

height = 0.40
lms  = 0.11
rms  = 0.97
hms  = 0.05

bms2 = 0.1
tms2 = bms2 + height
bms1 = tms2 + hms
tms1 = bms1 + height

set term postscript eps dashed enhanced color 
set output "bode_plot_comparison.eps"

set multiplot

set logscale x
set key left bottom reverse Left

set xr [0.1:100]

set xlabel "Frequency [Hz]"
set ylabel "Phase [rad]"
set lmargin screen lms
set rmargin screen rms
set tmargin screen tms2
set bmargin screen bms2

p "ref_bode.txt" u 1:3 w l lt -1 lw 2 ti "", \
  "zero_bode.txt" u 1:3 w l ls 1 ti "", \
  "est_bode.txt" u 1:3 w l ls 4 ti "" #, \
  #"fb_bode.txt"  u 1:3 w l ls 7 ti "" 


set xlabel ""
set ylabel "Magtinude"
set format x ""
set logscale y
set yr [0.005: 100]

set lmargin screen lms
set rmargin screen rms
set tmargin screen tms1
set bmargin screen bms1

p "ref_bode.txt" u 1:2 w l lt -1 lw 2 ti  "Model", \
  "zero_bode.txt" u 1:2 w l ls 1 ti "MRAC w/ zero initial parameters", \
  "est_bode.txt" u 1:2 w l ls 4 ti "MRAC w/ estimated initial parameters" # , \
  # "fb_bode.txt"  u 1:2 w l ls 7 ti "FB controller" 

unset multiplot
set output