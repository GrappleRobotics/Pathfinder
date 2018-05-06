set output "coupled.png"

set multiplot layout 2, 2 title "Coupled Drivetrain"
set title "Path / Velocity"
unset key
plot for [path in 'left center right'] sprintf("<(grep '^%s,' coupled.csv)", path) u 3:4:6 title path w l lc palette lw 3

set title "Simulated Path / Velocity"
unset key
plot for [path in 'left center right'] sprintf("<(grep '^%s,' coupled_simulated.csv)", path) u 3:4:5 title path w l lc palette lw 3

set title "Simulated vs Actual Path"
unset key
plot    for [path in 'left center right'] sprintf("<(grep '^%s,' coupled.csv)", path) u 3:4 title path w l lc "red" lw 3, \
        for [path in 'left center right'] sprintf("<(grep '^%s,' coupled_simulated.csv)", path) u 3:4 title path w l lc "green" lw 3

set title "Velocity"
set key autotitle columnhead
plot for [path in 'left center right'] sprintf("<(grep '^%s,' coupled.csv)", path) u 2:6 title path w l lw 3

unset multiplot