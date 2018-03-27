set datafile separator ","

set term x11 persist

set size square
plot for [path in 'left center right'] sprintf("<(grep '^%s,' ../build/test-results/kinematicsTest/coupled.csv)", path) u 3:4:6 title path w l lc palette lw 5