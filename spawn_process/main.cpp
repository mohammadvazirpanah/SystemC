
// #include "top.hpp"
#include "top1.hpp"

#include <systemc.h>

int sc_main(int argc, char **argv)
{
    top top1("top");
    sc_start();   
    return 0;
}

