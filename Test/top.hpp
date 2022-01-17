#define SC_INCLUDE_DYNAMIC_PROCESSES
#include <systemc.h>

int test_function(double d)
{
  cout << endl << "Test_function sees " << d << endl;
  return int(d);
}


class SelfAware : public sc_module
{
  private:


  public:

  SC_HAS_PROCESS(SelfAware);

  SelfAware(sc_module_name name) : sc_module(name)

    {
       
    SC_THREAD(main);
    cout << endl << sc_time_stamp() << ": CTOR, Before spawning!!!" << endl;
    }

    void main()
    {
    int i = 0;   
    sc_process_handle h = sc_spawn(&i ,sc_bind(&SelfAware::add_process, this));
    sc_process_handle h2 = sc_spawn(sc_bind(&SelfAware::delete_process, this,i));
    //     // sc_event e1, e2;
    //     // sc_fifo_in<int> in;
    //     // sc_fifo_out<int> out;
    //     // int r;
    //     // e1.notify();

    //     // sc_spawn(&r, sc_bind(&top::test, this, "report", sc_ref(e1), sc_ref(e2)), "1") ;
    //     // sc_spawn(&r, sc_bind(&top::test, this, "test", sc_ref(e2), sc_ref(e1)), "2") ;
 
    //   // for (int i = 0 ; i < 10; i++)
    //   //     sc_spawn(&r, sc_bind(&top::wait_and_end, this, i));

    //   // wait(20, SC_NS);

    //   add_process();

    }

  // int test(const char *str, sc_event& receive, sc_event& send)
  int test(int &i)

  {

      // wait(receive);
      cout << "This Message Was Received: " << i <<std::endl;
      wait(10, SC_NS);
      // send.notify();
    

    return i;
  }

    int wait_and_end(int i)
  {
    wait( i + 1, SC_NS);
    cout << "Thread " << i << " ending." << endl;
    return 0;
  }

  int add_process()
  {
    int i = 0;
    std::cout << "Process is Added" << std::endl;
    for (i = 0; i < 10; i++)
    {
      sc_spawn(sc_bind(&SelfAware::test, this,i));
    }
    std::cout<< "Process is Ended: " << i <<std::endl;
  return i;
  }

  void delete_process(int i)
  {
    wait(i + 1, SC_NS);
    std::cout << "Process is Deleted: " << i << std::endl;
  }
 
  
};

