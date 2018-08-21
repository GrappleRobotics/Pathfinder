set term png size 1800,1800 enhanced
set output "cdt.png"
set multiplot layout 3, 3 title "CDT Model"

set title "Accel"
plot "cdt_model.csv" using 1:2 w l lw 2, "" using 1:6 w l lw 2

set title "Velocity"
plot "cdt_model.csv" using 1:3 w l lw 2, "" using 1:7 w l lw 2

set title "Current"
plot "cdt_model.csv" using 1:4 w l lw 2, "" using 1:8 w l lw 2

set title "Voltage"
plot "cdt_model.csv" using 1:5 w l lw 2, "" using 1:9 w l lw 2

set title "Position Sim"
plot "cdt_model.csv" using 10:11 w l lw 2

set title "Angular Velocity"
plot "cdt_model.csv" using 1:12 w l lw 2

set title "Wheel Velocity"
plot "cdt_model.csv" using 1:13 w l lw 2, "" using 1:14 w l lw 2