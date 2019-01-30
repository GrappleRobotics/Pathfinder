set output "motor.png"

set multiplot

set title "CIM Motor"
set ytics 0.5
set yrange[0:3.5]
set ylabel "Torque" textcolor rgb "yellow"
plot "motor.csv" u 1:2

set ytics 50
set yrange[0:350]
set ytics offset -8, 0
set ylabel "Current" textcolor rgb "blue"
plot "motor.csv" u 1:3

unset multiplot