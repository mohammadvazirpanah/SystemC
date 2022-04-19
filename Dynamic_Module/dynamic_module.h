#ifndef SC_DYNAMIC_MODULE_H
#define SC_DYNAMIC_MODULE_H

#include "sysc/kernel/sc_kernel_ids.h"
#include "sysc/kernel/sc_process.h"
#include "sysc/kernel/sc_module_name.h"
#include "sysc/kernel/sc_sensitive.h"
#include "sysc/kernel/sc_time.h"
#include "sysc/kernel/sc_wait.h"
#include "sysc/kernel/sc_wait_cthread.h"
#include "sysc/kernel/sc_process.h"
#include "sysc/kernel/sc_process_handle.h"
#include "sysc/utils/sc_list.h"

namespace sc_core {

template<class T> class sc_in;
template<class T> class sc_inout;
template<class T> class sc_out;


struct SC_API sc_bind_proxy
{
    sc_interface* iface;
    sc_port_base* port;
    
    sc_bind_proxy();
    sc_bind_proxy( sc_interface& );
    sc_bind_proxy( sc_port_base& );
};


extern SC_API const sc_bind_proxy SC_BIND_PROXY_NIL;


class SC_API dynamic_module : public sc_object , sc_process_host {

    friend class sc_module_name;
    friend class sc_module_registry;
    friend class sc_object;
    friend class sc_port_registry;
	friend class sc_process_b;
    friend class sc_simcontext;

public:

    sc_simcontext* sc_get_curr_simcontext()
	{ return simcontext(); }

    const char* gen_unique_name( const char* basename_, bool preserve_first );

    virtual const char* kind() const { return "sc_module"; }

protected: 

    virtual void before_end_of_elaboration();

    void construction_done();

    virtual void end_of_elaboration();

    void elaboration_done( bool& );

    virtual void start_of_simulation();

    void start_simulation();

    virtual void end_of_simulation();

    void simulation_done();

    void sc_module_init();

    dynamic_module ();
    dynamic_module ( const sc_module_name& nm ); /* for those used to old style */
public:
    virtual ~dynamic_module();

    dynamic_module& operator << ( sc_interface& );
    dynamic_module& operator << ( sc_port_base& );

    dynamic_module& operator , ( sc_interface& interface_ )
        { return operator << ( interface_ ); }

    dynamic_module& operator , ( sc_port_base& port_ )
        { return operator << ( port_ ); }

    // operator() is declared at the end of the class.
    

    const ::std::vector<sc_object*>& get_child_objects() const;

protected:

    void end_module();

    void dont_initialize();

    void positional_bind( sc_interface& );
    void positional_bind( sc_port_base& );

    void async_reset_signal_is( const sc_in<bool>& port, bool level );
    void async_reset_signal_is( const sc_inout<bool>& port, bool level );
    void async_reset_signal_is( const sc_out<bool>& port, bool level );
    void async_reset_signal_is( const sc_signal_in_if<bool>& iface, bool level);
    void reset_signal_is( const sc_in<bool>& port, bool level );
    void reset_signal_is( const sc_inout<bool>& port, bool level );
    void reset_signal_is( const sc_out<bool>& port, bool level );
    void reset_signal_is( const sc_signal_in_if<bool>& iface, bool level );


    void wait( const sc_event& e )
        { ::sc_core::wait( e, simcontext() ); }

    void wait( const sc_event_or_list& el )
	{ ::sc_core::wait( el, simcontext() ); }

    void wait( const sc_event_and_list& el )
	{ ::sc_core::wait( el, simcontext() ); }

    void wait( const sc_time& t )
        { ::sc_core::wait( t, simcontext() ); }

    void wait( double v, sc_time_unit tu )
        { ::sc_core::wait( sc_time( v, tu, simcontext() ), simcontext() ); }

    void wait( const sc_time& t, const sc_event& e )
        { ::sc_core::wait( t, e, simcontext() ); }

    void wait( double v, sc_time_unit tu, const sc_event& e )
        { ::sc_core::wait( 
		sc_time( v, tu, simcontext() ), e, simcontext() ); }

    void wait( const sc_time& t, const sc_event_or_list& el )
        { ::sc_core::wait( t, el, simcontext() ); }

    void wait( double v, sc_time_unit tu, const sc_event_or_list& el )
        { ::sc_core::wait( sc_time( v, tu, simcontext() ), el, simcontext() ); }

    void wait( const sc_time& t, const sc_event_and_list& el )
        { ::sc_core::wait( t, el, simcontext() ); }

    void wait( double v, sc_time_unit tu, const sc_event_and_list& el )
        { ::sc_core::wait( sc_time( v, tu, simcontext() ), el, simcontext() ); }



    void next_trigger()
	{ ::sc_core::next_trigger( simcontext() ); }


    // dynamic sensitivty for SC_METHODs

    void next_trigger( const sc_event& e )
        { ::sc_core::next_trigger( e, simcontext() ); }

    void next_trigger( const sc_event_or_list& el )
        { ::sc_core::next_trigger( el, simcontext() ); }

    void next_trigger( const sc_event_and_list& el )
        { ::sc_core::next_trigger( el, simcontext() ); }

    void next_trigger( const sc_time& t )
        { ::sc_core::next_trigger( t, simcontext() ); }

    void next_trigger( double v, sc_time_unit tu )
        { ::sc_core::next_trigger( 
	    sc_time( v, tu, simcontext() ), simcontext() ); }

    void next_trigger( const sc_time& t, const sc_event& e )
        { ::sc_core::next_trigger( t, e, simcontext() ); }

    void next_trigger( double v, sc_time_unit tu, const sc_event& e )
        { ::sc_core::next_trigger( 
		sc_time( v, tu, simcontext() ), e, simcontext() ); }

    void next_trigger( const sc_time& t, const sc_event_or_list& el )
        { ::sc_core::next_trigger( t, el, simcontext() ); }

    void next_trigger( double v, sc_time_unit tu, const sc_event_or_list& el )
        { ::sc_core::next_trigger( 
	    sc_time( v, tu, simcontext() ), el, simcontext() ); }

    void next_trigger( const sc_time& t, const sc_event_and_list& el )
        { ::sc_core::next_trigger( t, el, simcontext() ); }

    void next_trigger( double v, sc_time_unit tu, const sc_event_and_list& el )
        { ::sc_core::next_trigger( 
	    sc_time( v, tu, simcontext() ), el, simcontext() ); }


    // for SC_METHODs and SC_THREADs and SC_CTHREADs

    bool timed_out()
        { return ::sc_core::timed_out(); }


    // for SC_CTHREADs

    void halt()
        { ::sc_core::halt( simcontext() ); }

    void wait( int n )
        { ::sc_core::wait( n, simcontext() ); }

    void at_posedge( const sc_signal_in_if<bool>& s )
	{ ::sc_core::at_posedge( s, simcontext() ); }

    void at_posedge( const sc_signal_in_if<sc_dt::sc_logic>& s )
	{ ::sc_core::at_posedge( s, simcontext() ); }

    void at_negedge( const sc_signal_in_if<bool>& s )
	{ ::sc_core::at_negedge( s, simcontext() ); }

    void at_negedge( const sc_signal_in_if<sc_dt::sc_logic>& s )
	{ ::sc_core::at_negedge( s, simcontext() ); }

    // Catch uses of watching:
    void watching( bool /* expr */ )
        { SC_REPORT_ERROR(SC_ID_WATCHING_NOT_ALLOWED_,""); }

    // These are protected so that user derived classes can refer to them.
    sc_sensitive     sensitive;
    sc_sensitive_pos sensitive_pos;
    sc_sensitive_neg sensitive_neg;

    // Function to set the stack size of the current (c)thread process.
    void set_stack_size( std::size_t );

    int append_port( sc_port_base* );


private:
    dynamic_module( const dynamic_module& );
    const dynamic_module& operator = ( const dynamic_module& );

private:

    bool                        m_end_module_called;
    std::vector<sc_port_base*>* m_port_vec;
    int                         m_port_index;
    sc_name_gen*                m_name_gen;
    sc_module_name*             m_module_name_p;

public:


void defunct() { }

    // positional binding methods (cont'd)

    void operator () ( const sc_bind_proxy& p001,
		       const sc_bind_proxy& p002 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p003 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p004 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p005 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p006 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p007 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p008 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p009 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p010 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p011 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p012 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p013 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p014 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p015 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p016 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p017 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p018 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p019 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p020 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p021 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p022 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p023 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p024 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p025 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p026 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p027 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p028 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p029 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p030 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p031 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p032 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p033 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p034 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p035 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p036 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p037 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p038 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p039 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p040 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p041 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p042 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p043 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p044 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p045 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p046 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p047 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p048 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p049 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p050 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p051 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p052 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p053 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p054 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p055 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p056 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p057 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p058 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p059 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p060 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p061 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p062 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p063 = SC_BIND_PROXY_NIL,
		       const sc_bind_proxy& p064 = SC_BIND_PROXY_NIL );

};  // class sc_dynamic_module

extern SC_API sc_module* sc_module_dynalloc(sc_module*);
#define SC_NEW(x)  ::sc_core::sc_module_dynalloc(new x);


#define SC_HAS_PROCESS(user_module_name)                                      \
    typedef user_module_name SC_CURRENT_USER_MODULE



#define declare_method_process(handle, name, host_tag, func)        \
    {		                                                    \
        ::sc_core::sc_process_handle handle =                      \
	    sc_core::sc_get_curr_simcontext()->create_method_process( \
		name,  false, SC_MAKE_FUNC_PTR( host_tag, func ), \
		this, 0 ); \
        this->sensitive << handle;                                        \
        this->sensitive_pos << handle;                                    \
        this->sensitive_neg << handle;                                    \
    }

#define declare_thread_process(handle, name, host_tag, func)        \
    {                                                               \
        ::sc_core::sc_process_handle handle =                      \
	     sc_core::sc_get_curr_simcontext()->create_thread_process( \
                 name,  false,           \
                 SC_MAKE_FUNC_PTR( host_tag, func ), this, 0 ); \
        this->sensitive << handle;                                        \
        this->sensitive_pos << handle;                                    \
        this->sensitive_neg << handle;                                    \
    }


#define SC_METHOD(func)                                                       \
    declare_method_process( func ## _handle,                                  \
                            #func,                                            \
                            SC_CURRENT_USER_MODULE,                           \
                            func )

#define SC_THREAD(func)                                                       \
    declare_thread_process( func ## _handle,                                  \
                            #func,                                            \
                            SC_CURRENT_USER_MODULE,                           \
                            func )


typedef dynamic_module sc_channel;
typedef dynamic_module sc_behavior;

} // namespace sc_core


#endif 