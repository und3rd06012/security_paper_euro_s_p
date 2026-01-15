set terminal pngcairo size 1000,800 font "Arial,12"
set datafile separator whitespace
set grid
set key top right

set output 'fig_attack_examples.png'
set multiplot layout 2,1 title 'ID3 static PID under reference jamming (500 Hz, A=0.25)'
set xlabel 'Time (s)'
set ylabel 'r(t), x(t)'
plot 'fig1.csv' using 1:2 with lines lc rgb 'red' lw 1 title 'r_a(t)', \
     'fig1.csv' using 1:3 with lines lc rgb 'blue' lw 1 title 'x(t)'
set ylabel 'u(t)'
plot 'fig1.csv' using 1:4 with lines lc rgb 'dark-green' lw 1 title 'u(t)'
unset multiplot

set output 'fig_escape_sweep.png'
set xlabel 'Time (s)'
set ylabel 'x(t)'
set yrange [-5:25]
plot 'fig2.csv' using 1:2 with lines lc rgb 'blue' lw 1 title 'ID6 lead-int, bias -5', \
     'fig2.csv' using 1:3 with lines lc rgb 'red' lw 1 title 'ID1 bang-bang, bias -5', \
     20 with lines lc rgb 'black' lw 2 dashtype 2 title 'safety bound |x|=20'
set yrange [*:*]

set output 'fig_mitigation.png'
set multiplot layout 2,1 title 'Mitigations'
set xlabel 'Time (s)'
set ylabel 'u(t)'
plot 'fig3a.csv' using 1:2 with lines lc rgb 'red' lw 1 title 'ID3, no mitigation', \
     'fig3a.csv' using 1:3 with lines lc rgb 'blue' lw 1 title 'ID3, with M1 rate gate'
set ylabel 'x(t)'
set yrange [-5:25]
plot 'fig3b.csv' using 1:2 with lines lc rgb 'red' lw 1 title 'ID6, no mitigation', \
     'fig3b.csv' using 1:3 with lines lc rgb 'blue' lw 1 title 'ID6, with M2 CUSUM+latch', \
     20 with lines lc rgb 'black' lw 2 dashtype 2 title 'safety bound'
unset multiplot
