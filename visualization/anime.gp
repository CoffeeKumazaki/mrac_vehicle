file = "./data/output/no_init_param/adaptive_track_vx_05_gain_10.csv"
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

set terminal gif animate delay 5 optimize size 640,480 font "Courier, 12"
set notics
set output 'path_following_track_vx_06_gain_10.gif'
set size square
# track
width = 2.3
height = 8.99

# vehicle
width = 1.76
height = 4.575
range = 20

do for [i=1: N] {
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
  set yrange [cy-range:cy+range]

  plot file every ::::i-1 using 1:2 w l ti "Vehicle", \
       pathfile w l lt 0

}

unset output
reset