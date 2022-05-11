#define SC_INCLUDE_DYNAMIC_PROCESSES
#include <systemc.h>

using namespace sc_core;
class actor : public sc_module
{
public:
  typedef std::function<void()> functype;
  // std::sc_vector< sc_signal<T> >, N >;

  actor(const sc_module_name &_name,
        const functype &_func) : sc_module(_name), _func(_func)
  {
    SC_THREAD(worker);
  }

private:
  sc_process_handle process;

  void worker()
  {
    add_actor();
  }

  functype _func;
  SC_HAS_PROCESS(actor);

public:
  void add_actor()
  {
    std::cout << "Actor: " << name() << " Is Added! " << endl;
    process = sc_spawn(sc_bind(&actor::_func, this));
  }

  // void add_actor(functype func)
  // {
  //   sc_process_handle process;
  //   process = sc_spawn(sc_bind(func, this));
  // }

  void del_actor()
  {
    process.kill();
  }

  void show_actors()
  {
    std::vector<sc_object *> vec = sc_get_current_process_handle().get_child_objects();
    for (long unsigned int i = 0; i < vec.size(); i++)
    {
      std::cout << vec[i]->name() << ", " << vec[i]->kind() << endl;
    }
  }

  void add_signal()
  {
  }

  void del_signal()
  {
  }
};
