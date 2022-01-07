CPP="$@"
SIM="sim"
g++ -I. -I$SYSTEMC_HOME/include -L. -L$SYSTEMC_HOME/lib-linux64 -o "$SIM" "$CPP" -lsystemc -lm
./$SIM
