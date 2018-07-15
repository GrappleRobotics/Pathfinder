set term png size 3000,1200 enhanced

set output "coupled.png"

set multiplot layout 2, 5 title "Coupled Drivetrain"
set title "Path / Velocity"
unset key
plot for [path in 'left center right'] sprintf("<(grep '^%s,' coupled.csv)", path) u 3:4:6 title path w l lc palette lw 3

set title "Simulated Path / Velocity"
unset key
plot for [path in 'left center right'] sprintf("<(grep '^%s,' coupled_simulated.csv)", path) u 4:5:6 title path w l lc palette lw 3

set title "Simulated vs Actual Path"
unset key
plot    for [path in 'left center right'] sprintf("<(grep '^%s,' coupled.csv)", path) u 3:4 title path w l lc "red" lw 3, \
        for [path in 'left center right'] sprintf("<(grep '^%s,' coupled_simulated.csv)", path) u 4:5 title path w l lc "green" lw 3

set title "Velocity"
set key autotitle columnhead
plot for [path in 'left center right'] sprintf("<(grep '^%s,' coupled.csv)", path) u 2:6 title path w l lw 3

set title "Simulated vs Actual Velocity"
unset key
plot    for [path in 'left center right'] sprintf("<(grep '^%s,' coupled.csv)", path) u 2:6 title path w l lc "red" lw 3, \
        for [path in 'left center right'] sprintf("<(grep '^%s,' coupled_simulated.csv)", path) u 3:6 title path w l lc "green" lw 3

set title "Angle"
unset key
plot    for [path in 'center'] sprintf("<(grep '^%s,' coupled.csv)", path) u 2:($8*180/3.1415) title path w l lc "red" lw 3

set title "Angular Velocity"
unset key
plot    for [path in 'center'] sprintf("<(grep '^%s,' coupled.csv)", path) u 2:9 title path w l lc "red" lw 3
        # for [path in 'center'] sprintf("<(grep '^%s,' coupled.csv)", path) u 2:($12/3) title path w l lc "green" lw 3

set title "Angular Acceleration"
unset key
plot    for [path in 'center'] sprintf("<(grep '^%s,' coupled.csv)", path) u 2:10 title path w l lc "red" lw 3

set title "Acceleration"
set key autotitle columnhead
plot for [path in 'center'] sprintf("<(grep '^%s,' coupled.csv)", path) u 2:7 title path w l lw 3

set title "Curvature"
set key autotitle columnhead
plot for [path in 'center'] sprintf("<(grep '^%s,' coupled.csv)", path) u 2:11 title path w l lw 3
unset multiplot