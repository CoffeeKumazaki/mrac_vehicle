model_file = "../data/output/with_init_param/prius_deadzone_0.0_vx_20_gain_00_straight_u04_005sin05_noise_0.0_mass200.csv"
## no adaptation
# plant_file = "../data/output/no_init_param/vehicle_deadzone_0.0_vx_20_gain_00_straight_u04_005sin05_noise_0.0_noadapt.csv"
## with adaptation
# plant_file = "../data/output/no_init_param/vehicle_deadzone_0.0_vx_20_gain_1_straight_u04_005sin05_noise_0.0.csv" 

## no adaptation
plant_file = "../data/output/no_init_param/small_deadzone_0.0_vx_20_gain_00_straight_u04_005sin05_noise_0.0_noadapt.csv"
## with adaptation
plant_file = "../data/output/with_init_param/small_deadzone_0.0_vx_20_gain_1_straight_u04_005sin05_noise_0.0.csv"

pathfile="./data/path.txt"
reset
stats plant_file using 1 nooutput
N = STATS_records

array ax[N]
array ay[N]
array aphi[N]
array px[N]
array py[N]
array pphi[N]

stats model_file using (ax[$0+1] = $1, 0) nooutput
stats model_file using (ay[$0+1] = $2, 0) nooutput
stats model_file using (aphi[$0+1] = $3, 0) nooutput

stats plant_file using (px[$0+1] = $1, 0) nooutput
stats plant_file using (py[$0+1] = $2, 0) nooutput
stats plant_file using (pphi[$0+1] = $3, 0) nooutput

set terminal gif animate delay 1 optimize size 640,480 font "Courier, 12"
set noytics
set format x ""
# set notics
set output 'small_deadzone_0.0_vx_20_gain_1_straight_u04_005sin05_noise_0.0.gif'
# track
width = 2.3
height = 8.99

# vehicle
width = 1.76
height = 4.575

# Nissan Cube
pwidth = 1.695
pheight = 3.89
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
  set arrow 1 nohead from cx+(x1*cos(phi) - y1*sin(phi)),cy+(x1*sin(phi) + y1*cos(phi)) to cx+(x2*cos(phi) - y2*sin(phi)),cy+(x2*sin(phi) + y2*cos(phi)) lc "blue"
  x1 = height/2
  y1 = width/2
  x2 = height/2
  y2 = -width/2
  set arrow 2 nohead from cx+(x1*cos(phi) - y1*sin(phi)),cy+(x1*sin(phi) + y1*cos(phi)) to cx+(x2*cos(phi) - y2*sin(phi)),cy+(x2*sin(phi) + y2*cos(phi)) lc "blue"
  x1 = -height/2
  y1 = -width/2
  x2 = -height/2
  y2 = width/2
  set arrow 3 nohead from cx+(x1*cos(phi) - y1*sin(phi)),cy+(x1*sin(phi) + y1*cos(phi)) to cx+(x2*cos(phi) - y2*sin(phi)),cy+(x2*sin(phi) + y2*cos(phi)) lc "blue"
  x1 = -height/2
  y1 = -width/2
  x2 = height/2
  y2 = -width/2
  set arrow 4 nohead from cx+(x1*cos(phi) - y1*sin(phi)),cy+(x1*sin(phi) + y1*cos(phi)) to cx+(x2*cos(phi) - y2*sin(phi)),cy+(x2*sin(phi) + y2*cos(phi)) lc "blue"

  cx = px[i]
  cy = py[i]
  phi = pphi[i]
  x1 = pheight/2
  y1 = pwidth/2
  x2 = -pheight/2
  y2 = pwidth/2
  set arrow 5 nohead from cx+(x1*cos(phi) - y1*sin(phi)),cy+(x1*sin(phi) + y1*cos(phi)) to cx+(x2*cos(phi) - y2*sin(phi)),cy+(x2*sin(phi) + y2*cos(phi)) lc "red"
  x1 = pheight/2
  y1 = pwidth/2
  x2 = pheight/2
  y2 = -pwidth/2
  set arrow 6 nohead from cx+(x1*cos(phi) - y1*sin(phi)),cy+(x1*sin(phi) + y1*cos(phi)) to cx+(x2*cos(phi) - y2*sin(phi)),cy+(x2*sin(phi) + y2*cos(phi)) lc "red"
  x1 = -pheight/2
  y1 = -pwidth/2
  x2 = -pheight/2
  y2 = pwidth/2
  set arrow 7 nohead from cx+(x1*cos(phi) - y1*sin(phi)),cy+(x1*sin(phi) + y1*cos(phi)) to cx+(x2*cos(phi) - y2*sin(phi)),cy+(x2*sin(phi) + y2*cos(phi)) lc "red"
  x1 = -pheight/2
  y1 = -pwidth/2
  x2 = pheight/2
  y2 = -pwidth/2
  set arrow 8 nohead from cx+(x1*cos(phi) - y1*sin(phi)),cy+(x1*sin(phi) + y1*cos(phi)) to cx+(x2*cos(phi) - y2*sin(phi)),cy+(x2*sin(phi) + y2*cos(phi)) lc "red"


  set xrange [cx-range:cx+range]
  set yrange [cy-range/2.0:cy+range/2.0]
  set origin 0, 0
  set size 1, 0.75
  set bmargin 2

  plot model_file every ::::i-1 using 1:2 w l ti "Model Trajectory" lc "blue", \
      plant_file every ::::i-1 using 1:2 w l ti "Plant Trajectory" lc "red", \
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
  unset arrow 5
  unset arrow 6
  unset arrow 7
  unset arrow 8
  set bmargin 0

  plot model_file every ::::i-1 using 1:2 w l ti "" lc "blue" ,\
       plant_file every ::::i-1 using 1:2 w l ti "" lc "red", \
       0 lt 0 ti "", 2 lt -1 ti "", -2 lt -1 ti ""

  set nomultiplot
}

unset output
reset