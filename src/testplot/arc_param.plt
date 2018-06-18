set term png size 1200,600 enhanced
set output "arc_param.png"
set multiplot layout 1, 2 title "Arc Parameterization"

set autoscale

set title "Path"
plot "arcparam.csv" using 3:4:1 w p ps 0.2 pt 7 lc palette lw 2

set yrange [0:1]
set autoscale x
set autoscale ymax

set title "Curvature"
plot "arcparam.csv" using 2:5:1 w l lc palette lw 2

unset multiplot