#include <systemc.h>
#include "sysc/kernel/dynamic_module.h" 


// SC_MODULE(top)
// {

// void Dynamic()
//     {
        
//     }

//     SC_CTOR(top)
//     {
//         SC_THREAD(Dynamic);
//     }
// };

class dy : public dynamic_module
{   
    private:


    public:

    SC_HAS_PROCESS(dy);

    dy(sc_module_name name) : dynamic_module(name)
    {
        SC_THREAD(Dynamic);
    }

    void Dynamic()
    {
        cout << "Dynamic" << endl;
    }
};





// class dy : public sc_module
// {   
//     private:


//     public:

//     SC_HAS_PROCESS(dy);
    
//     dy(sc_module_name name) : sc_module(name)
//     {
//         SC_THREAD(Dynamic);
//     }

//     void Dynamic()
//     {
//         cout << "Dynamic" << endl;
//     }
// };

