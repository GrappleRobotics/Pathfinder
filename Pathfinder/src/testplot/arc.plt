set term png size 1800,600 enhanced
set output "arc.png"
set multiplot layout 1, 3 title "Arc"

set xrange [-1:6]
set yrange [0:7]

set title "Path"
plot "arc.csv" using 2:3:1 w l lc palette lw 2

set title "Derivatives"
plot "arc.csv" using 2:3:4:5 w vectors head filled lt 2

set yrange [0:1]
set autoscale x
set autoscale ymax

set title "Curvature"
plot "arc.csv" using 1:6 w l lw 2, '' using 1:(1.0/$6) w l lw 2 title "Radius"

unset multiplot