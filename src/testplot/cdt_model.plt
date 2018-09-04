set term png size 1800,1800 enhanced
set output "cdt.png"
set multiplot layout 3, 3 title "CDT Model (Centre)"

set xrange [-2:6]
set yrange [-2:6]

set title "Position"
plot "cdt.csv" using 2:3:9 w p pt 7 lc palette

set xrange [*:*]
set yrange [*:*]

set title "Heading (Deg)"
plot "cdt.csv" using 1:($4*180/3.1415):9 w p pt 7 lc palette

set title "Distance"
plot "cdt.csv" using 1:5:9 w p pt 7 lc palette

set title "Velocity"
plot "cdt.csv" using 1:6:9 w p pt 7 lc palette

set title "Acceleration"
plot "cdt.csv" using 1:7:9 w p pt 7 lc palette ps 0.5

set title "Curvature"
plot "cdt.csv" using 1:8:9 w p pt 7 lc palette

set title "Voltage"
plot "cdt.csv" using 1:10:9 w p pt 7 lc palette

set title "Current"
plot "cdt.csv" using 1:11:9 w p pt 7 lc palette 