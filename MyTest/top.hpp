#include <systemc.h>

SC_MODULE(Mytest)
{

void prnt()
    {
        std::cout<<"My Second Module\n";

    }

    SC_CTOR(Mytest)
    {
        //SC_THREAD(prnt);
    }
};

SC_MODULE(top)
{

void test()
    {
        std::cout<<"My First Module\n";
        wait(10,SC_NS);
        Mytest test1 ("test1");

    }

    SC_CTOR(top)
    {
        SC_THREAD(test);
    }
};
