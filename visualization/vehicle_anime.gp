# file = "../data/output/with_init_param/prius_deadzone_0.0_vx_20_gain_00_straight_u04_005sin05_noise_0.0_mass200.csv"
file = "../data/output/with_init_param/vehicle_deadzone_0.0_vx_20_gain_1_straight_u04_005sin05_noise_0.0.csv"
pathfile="./data/path.txt"
reset
stats file using 1 nooutput
N = STATS_records

array ax[N]
array ay[N]
array aphi[N]

stats file using (ax[$0+1] = $1, 0) nooutput
stats file using (ay[$0+1] = $2, 0) nooutput
stats file using (aphi[$0+1] = $3, 0) nooutput

set terminal gif animate delay 1 optimize size 640,480 font "Courier, 12"
set noytics
set format x ""
# set notics
set output 'vehicle_deadzone_0.0_vx_20_gain_1_straight_u04_005sin05_noise_0.0.gif'
# track
width = 2.3
height = 8.99

# vehicle
width = 1.76
height = 4.575
range = 20

do for [i=1: N: 2] {
  set multiplot

  cx = ax[i]
  cy = ay[i]
  phi = aphi[i]
  x1 = height/2
  y1 = width/2
  x2 = -height/2
  y2 = width/2
  set arrow 1 nohead from cx+(x1*cos(phi) - y1*sin(phi)),cy+(x1*sin(phi) + y1*cos(phi)) to cx+(x2*cos(phi) - y2*sin(phi)),cy+(x2*sin(phi) + y2*cos(phi))
  x1 = height/2
  y1 = width/2
  x2 = height/2
  y2 = -width/2
  set arrow 2 nohead from cx+(x1*cos(phi) - y1*sin(phi)),cy+(x1*sin(phi) + y1*cos(phi)) to cx+(x2*cos(phi) - y2*sin(phi)),cy+(x2*sin(phi) + y2*cos(phi))
  x1 = -height/2
  y1 = -width/2
  x2 = -height/2
  y2 = width/2
  set arrow 3 nohead from cx+(x1*cos(phi) - y1*sin(phi)),cy+(x1*sin(phi) + y1*cos(phi)) to cx+(x2*cos(phi) - y2*sin(phi)),cy+(x2*sin(phi) + y2*cos(phi))
  x1 = -height/2
  y1 = -width/2
  x2 = height/2
  y2 = -width/2
  set arrow 4 nohead from cx+(x1*cos(phi) - y1*sin(phi)),cy+(x1*sin(phi) + y1*cos(phi)) to cx+(x2*cos(phi) - y2*sin(phi)),cy+(x2*sin(phi) + y2*cos(phi))

  set xrange [cx-range:cx+range]
  set yrange [cy-range/2.0:cy+range/2.0]
  set origin 0, 0
  set size 1, 0.75
  set bmargin 2

  plot file every ::::i-1 using 1:2 w l ti "Trajectory" lc "blue", \
      0 lt 0 ti "", 2 lt -1 ti "", -2 lt -1 ti ""
  #      pathfile w l lt 0

  set xrange [0:1800]
  set yrange [-2.5:2.5]
  set origin 0, 0.75
  set size 1, 0.25

  unset arrow 1
  unset arrow 2
  unset arrow 3
  unset arrow 4
  set bmargin 0

  plot file every ::::i-1 using 1:2 w l ti "" lc "blue" ,\
      0 lt 0 ti "", 2 lt -1 ti "", -2 lt -1 ti ""

  set nomultiplot
}

unset output
reset