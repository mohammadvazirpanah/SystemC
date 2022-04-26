# SystemC
## Run Command
``` g++ main.cpp -I. -I/.../systemc-2.3.3/include -L. -L"/.../systemc-2.3.3/lib-linux64" -Wl,-rpath=/.../systemc-2.3.3/lib-linux64  -o main -lsystemc -lm ```
## Run Command with Using roscpp Library
``` g++ main.cpp -I. -I/.../systemc-2.3.3/include -L. -L"/.../systemc-2.3.3/lib-linux64" -I/opt/ros/noetic/include -L/opt/ros/noetic/lib -Wl,-rpath=/opt/ros/noetic/lib -Wl,-rpath=/.../systemc-2.3.3/lib-linux64  -o main -lroscpp -lrosconsole -lrostime -lroscpp_serialization -lsystemc -lm ```

