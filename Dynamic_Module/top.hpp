#include <systemc.h>


SC_MODULE(top)
{

void Dynamic()
    {
        
    }

    SC_CTOR(top)
    {
        SC_THREAD(Dynamic);
    }
};


