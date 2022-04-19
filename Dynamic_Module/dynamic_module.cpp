/*****************************************************************************

  Licensed to Accellera Systems Initiative Inc. (Accellera) under one or
  more contributor license agreements.  See the NOTICE file distributed
  with this work for additional information regarding copyright ownership.
  Accellera licenses this file to you under the Apache License, Version 2.0
  (the "License"); you may not use this file except in compliance with the
  License.  You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
  implied.  See the License for the specific language governing
  permissions and limitations under the License.

 *****************************************************************************/

/*****************************************************************************

  sc_module.cpp -- Base class of all sequential and combinational processes.

  Original Author: Stan Y. Liao, Synopsys, Inc.

  CHANGE LOG AT THE END OF THE FILE
 *****************************************************************************/


#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <sstream>

#include "sysc/kernel/sc_event.h"
#include "sysc/kernel/sc_kernel_ids.h"
#include "sysc/kernel/dynamic_module.h"
#include "sysc/kernel/sc_module_registry.h"
#include "sysc/kernel/sc_name_gen.h"
#include "sysc/kernel/sc_object_manager.h"
#include "sysc/kernel/sc_process.h"
#include "sysc/kernel/sc_process_handle.h"
#include "sysc/kernel/sc_simcontext.h"
#include "sysc/kernel/sc_simcontext_int.h"
#include "sysc/kernel/sc_object_int.h"
#include "sysc/kernel/sc_reset.h"
#include "sysc/communication/sc_communication_ids.h"
#include "sysc/communication/sc_interface.h"
#include "sysc/communication/sc_port.h"
#include "sysc/communication/sc_signal.h"
#include "sysc/communication/sc_signal_ports.h"
#include "sysc/utils/sc_utils_ids.h"

namespace sc_core {

// ----------------------------------------------------------------------------
//  CLASS : sc_module_dynalloc_list
//
//  Garbage collection for modules dynamically allocated with SC_NEW.
// ----------------------------------------------------------------------------

class sc_module_dynalloc_list
{
public:

    sc_module_dynalloc_list() : m_list()
        {}

    ~sc_module_dynalloc_list();

    void add( dynamic_module* p )
        { m_list.push_back( p ); }

private:

    sc_plist<dynamic_module*> m_list;
};


//------------------------------------------------------------------------------
//"~sc_module_dynalloc_list"
//
// Note we clear the m_parent field for the module being deleted. This because
// we process the list front to back so the parent has already been deleted, 
// and we don't want ~sc_object() to try to access the parent which may 
// contain garbage.
//------------------------------------------------------------------------------
sc_module_dynalloc_list::~sc_module_dynalloc_list()
{
    sc_plist<sc_module*>::iterator it( m_list );
    while( ! it.empty() ) {
        (*it)->m_parent = 0;
        delete *it;
        it ++;
    }
}


// ----------------------------------------------------------------------------

SC_API dynamic_module*
sc_module_dynalloc( dynamic_module* module_ )
{
    static sc_module_dynalloc_list dynalloc_list;
    dynalloc_list.add( module_ );
    return module_;
}


// ----------------------------------------------------------------------------
//  STRUCT : sc_bind_proxy
//
//  Struct for temporarily storing a pointer to an interface or port.
//  Used for positional binding.
// ----------------------------------------------------------------------------
    
sc_bind_proxy::sc_bind_proxy()
: iface( 0 ),
  port( 0 )
{}

sc_bind_proxy::sc_bind_proxy( sc_interface& iface_ )
: iface( &iface_ ),
  port( 0 )
{}

sc_bind_proxy::sc_bind_proxy( sc_port_base& port_ )
: iface( 0 ),
  port( &port_ )
{}


SC_API const sc_bind_proxy SC_BIND_PROXY_NIL;


// ----------------------------------------------------------------------------
//  CLASS : sc_module
//
//  Base class for all structural entities.
// ----------------------------------------------------------------------------

void
dynamic_module::sc_module_init()
{
    simcontext()->get_module_registry()->insert( *this );
    simcontext()->hierarchy_push( this );
    m_end_module_called = false;
    m_module_name_p = 0;
    m_port_vec = new std::vector<sc_port_base*>;
    m_port_index = 0;
}

/*
 *  This form of the constructor assumes that the user has
 *  used an sc_module_name parameter for his/her constructor.
 *  That parameter object has been pushed onto the stack,
 *  and can be looked up by calling the 
 *  top_of_module_name_stack() function of the object manager.
 *  This technique has two advantages:
 *
 *  1) The user no longer has to write sc_module(name) in the
 *     constructor initializer.
 *  2) The user no longer has to call end_module() at the end
 *     of the constructor -- a common negligence.
 *
 *  But it is important to note that sc_module_name may be used
 *  in the user's constructor's parameter. If it is used anywhere
 *  else, unexpected things will happen. The error-checking
 *  mechanism builtin here cannot hope to catch all misuses.
 *
 */

dynamic_module::dynamic_module()
: sc_object(::sc_core::sc_get_curr_simcontext()
                  ->get_object_manager()
                  ->top_of_module_name_stack()
                  ->operator const char*()),
  sensitive(this),
  sensitive_pos(this),
  sensitive_neg(this),
  m_end_module_called(false),
  m_port_vec(),
  m_port_index(0),
  m_name_gen(0),
  m_module_name_p(0)
{
    /* When this form is used, we better have a fresh sc_module_name
       on the top of the stack */
    sc_module_name* mod_name = 
        simcontext()->get_object_manager()->top_of_module_name_stack();
    if (0 == mod_name || 0 != mod_name->m_module_p) {
        SC_REPORT_ERROR( SC_ID_SC_MODULE_NAME_REQUIRED_, 0 );
        sc_abort(); // can't recover from here
    }
    sc_module_init();
    mod_name->set_module( this );
    m_module_name_p = mod_name; // must come after sc_module_init call.
}

dynamic_module::sc_module( const sc_module_name& )
: sc_object(::sc_core::sc_get_curr_simcontext()
                  ->get_object_manager()
                  ->top_of_module_name_stack()
                  ->operator const char*()),
  sensitive(this),
  sensitive_pos(this),
  sensitive_neg(this),
  m_end_module_called(false),
  m_port_vec(),
  m_port_index(0),
  m_name_gen(0),
  m_module_name_p(0)
{
    /* For those used to the old style of passing a name to sc_module,
       this constructor will reduce the chance of making a mistake */

    /* When this form is used, we better have a fresh sc_module_name
       on the top of the stack */
    sc_module_name* mod_name = 
        simcontext()->get_object_manager()->top_of_module_name_stack();
    if (0 == mod_name || 0 != mod_name->m_module_p) {
        SC_REPORT_ERROR( SC_ID_SC_MODULE_NAME_REQUIRED_, 0 );
        sc_abort(); // can't recover from here
    }
    sc_module_init();
    mod_name->set_module( this );
    m_module_name_p = mod_name; // must come after sc_module_init call.
}

/* --------------------------------------------------------------------
 *
 * Deprecated constructors:
 *   sc_module( const char* )
 *   sc_module( const std::string& )
 */
dynamic_module::dynamic_module( const char* nm )
: sc_object(nm),
  sensitive(this),
  sensitive_pos(this),
  sensitive_neg(this),
  m_end_module_called(false),
  m_port_vec(),
  m_port_index(0),
  m_name_gen(0),
  m_module_name_p(0)
{
    SC_REPORT_WARNING( SC_ID_BAD_SC_MODULE_CONSTRUCTOR_, nm );
    sc_module_init();
}

dynamic_module::sc_module( const std::string& s )
: sc_object( s.c_str() ),
  sensitive(this),
  sensitive_pos(this),
  sensitive_neg(this),
  m_end_module_called(false),
  m_port_vec(),
  m_port_index(0),
  m_name_gen(0),
  m_module_name_p(0)
{
    SC_REPORT_WARNING( SC_ID_BAD_SC_MODULE_CONSTRUCTOR_, s.c_str() );
    sc_module_init();
}

/* -------------------------------------------------------------------- */

dynamic_module::~dynamic_module()
{
    delete m_port_vec;
    delete m_name_gen;
    orphan_child_objects();
    if ( m_module_name_p )
    {
	m_module_name_p->clear_module( this ); // must be before end_module()
    	end_module();
    }
    simcontext()->get_module_registry()->remove( *this );
}


const ::std::vector<sc_object*>&
dynamic_module::get_child_objects() const
{
    return m_child_objects;
}

// set SC_THREAD asynchronous reset sensitivity

void
dynamic_module::async_reset_signal_is( const sc_in<bool>& port, bool level )
{
	sc_reset::reset_signal_is(true, port, level);
}

void
dynamic_module::async_reset_signal_is( const sc_inout<bool>& port, bool level )
{
	sc_reset::reset_signal_is(true, port, level);
}

void
dynamic_module::async_reset_signal_is( const sc_out<bool>& port, bool level )
{
	sc_reset::reset_signal_is(true, port, level);
}

void
dynamic_module::async_reset_signal_is(const sc_signal_in_if<bool>& iface, bool level)
{
	sc_reset::reset_signal_is(true, iface, level);
}

void
dynamic_module::end_module()
{
    if( ! m_end_module_called ) {
	/* TBD: Can check here to alert the user that end_module
                was not called for a previous module. */
	(void)sc_get_curr_simcontext()->hierarchy_pop();
	sc_get_curr_simcontext()->reset_curr_proc(); 
	sensitive.reset();
	sensitive_pos.reset();
	sensitive_neg.reset();
	m_end_module_called = true;
	m_module_name_p = 0; // make sure we are not called in ~sc_module().
    }
}


// to prevent initialization for SC_METHODs and SC_THREADs

void
dynamic_module::dont_initialize()
{
    sc_process_handle last_proc = sc_get_last_created_process_handle();
    last_proc.dont_initialize( true );
}

// set SC_THREAD synchronous reset sensitivity

void
dynamic_module::reset_signal_is( const sc_in<bool>& port, bool level )
{
	sc_reset::reset_signal_is(false, port, level);
}

void
dynamic_module::reset_signal_is( const sc_inout<bool>& port, bool level )
{
	sc_reset::reset_signal_is(false, port, level);
}

void
dynamic_module::reset_signal_is( const sc_out<bool>& port, bool level )
{
	sc_reset::reset_signal_is(false, port, level);
}

void
dynamic_module::reset_signal_is( const sc_signal_in_if<bool>& iface, bool level )
{
	sc_reset::reset_signal_is(false, iface, level);
}

// to generate unique names for objects in an MT-Safe way

const char*
dynamic_module::gen_unique_name( const char* basename_, bool preserve_first )
{
    if( !m_name_gen ) m_name_gen = new sc_name_gen;
    return m_name_gen->gen_unique_name( basename_, preserve_first );
}


// called by construction_done 

void
dynamic_module::before_end_of_elaboration()
{}

// We push the sc_module instance onto the stack of open objects so 
// that any objects that are created in before_end_of_elaboration have
// the proper parent. After the call we pop the hierarchy.
void
dynamic_module::construction_done()
{
    hierarchy_scope scope(this);
    before_end_of_elaboration();
}

// called by elaboration_done (does nothing by default)

void
dynamic_module::end_of_elaboration()
{}


// We push the sc_module instance onto the stack of open objects so 
// that any objects that are created in end_of_elaboration have
// the proper parent. After the call we pop the hierarchy.
void
dynamic_module::elaboration_done( bool& error_ )
{
    if( ! m_end_module_called ) {
        std::stringstream msg;
        msg << "module '" << name() << "'";
        SC_REPORT_WARNING( SC_ID_END_MODULE_NOT_CALLED_, msg.str().c_str() );
        if( error_ ) {
            SC_REPORT_WARNING( SC_ID_HIER_NAME_INCORRECT_, 0 );
        }
        error_ = true;
    }
    hierarchy_scope scope(this);
    end_of_elaboration();
}

// called by start_simulation (does nothing by default)

void
dynamic_module::start_of_simulation()
{}

void
dynamic_module::start_simulation()
{
    hierarchy_scope scope(this);
    start_of_simulation();
}

// called by simulation_done (does nothing by default)

void
dynamic_module::end_of_simulation()
{}

void
dynamic_module::simulation_done()
{
    hierarchy_scope scope(this);
    end_of_simulation();
}

void
dynamic_module::set_stack_size( std::size_t size )
{
    sc_process_handle  proc_h(
    	sc_is_running() ?
	sc_get_current_process_handle() :
	sc_get_last_created_process_handle()
    );
    sc_thread_handle thread_h;  // Current process as thread.


    thread_h = (sc_thread_handle)proc_h;
    if ( thread_h ) 
    {
	thread_h->set_stack_size( size );
    }
    else
    {
	SC_REPORT_WARNING( SC_ID_SET_STACK_SIZE_, 0 );
    }
}


int
dynamic_module::append_port( sc_port_base* port_ )
{
    int index = m_port_vec->size();
    m_port_vec->push_back( port_ );
    return index;
}


// positional binding methods

static void sc_warn_arrow_arrow_bind()
{
    static bool warn_arrow_arrow_bind=true;
    if ( warn_arrow_arrow_bind )
    {
    	warn_arrow_arrow_bind = false;
	SC_REPORT_INFO(SC_ID_IEEE_1666_DEPRECATION_,
	    "positional binding using << or , is deprecated, use () instead.");
    }
}

dynamic_module&
dynamic_module::operator << ( sc_interface& interface_ )
{
    sc_warn_arrow_arrow_bind();
    positional_bind(interface_);
    return *this;
}

dynamic_module&
dynamic_module::operator << ( sc_port_base& port_ )
{
    sc_warn_arrow_arrow_bind();
    positional_bind(port_);
    return *this;
}


void
dynamic_module::positional_bind( sc_interface& interface_ )
{
    if( m_port_index == (int)m_port_vec->size() ) {
        std::stringstream msg;
        if( m_port_index == 0 ) {
            msg << "module `" << name() << "' has no ports";
        } else {
            msg << "all ports of module `" << name() << "' are bound";
        }
        SC_REPORT_ERROR( SC_ID_BIND_IF_TO_PORT_, msg.str().c_str() );
        return;
    }
    int status = (*m_port_vec)[m_port_index]->pbind( interface_ );
    if( status != 0 ) {
        std::stringstream msg;
        switch( status ) {
        case 1:
            msg << "port " << m_port_index
                << " of module `" << name() << "' is already bound";
            break;
        case 2:
            msg << "type mismatch on port " << m_port_index
                << " of module `" << name() << "'";
            break;
        default:
            msg << "unknown error";
            break;
        }
        SC_REPORT_ERROR( SC_ID_BIND_IF_TO_PORT_, msg.str().c_str() );
    }
    ++ m_port_index;
}

void
dynamic_module::positional_bind( sc_port_base& port_ )
{
    if( m_port_index == (int)m_port_vec->size() ) {
        std::stringstream msg;
        if( m_port_index == 0 ) {
            msg << "module `" << name() << "' has no ports";
        } else {
            msg << "all ports of module `" << name() << "' are bound";
        }
        SC_REPORT_ERROR( SC_ID_BIND_IF_TO_PORT_, msg.str().c_str() );
        return;
    }
    int status = (*m_port_vec)[m_port_index]->pbind( port_ );
    if( status != 0 ) {
        std::stringstream msg;
        switch( status ) {
        case 1:
            msg << "port " << m_port_index
                << " of module `" << name() << "' is already bound";
            break;
        case 2:
            msg << "type mismatch on port " << m_port_index
                << " of module `" << name() << "'";
            break;
        default:
            msg << "unknown error";
            break;
        }
        SC_REPORT_ERROR( SC_ID_BIND_IF_TO_PORT_, msg.str().c_str() );
    }
    ++ m_port_index;
}


#define TRY_BIND( p )                                                         \
    if( (p).iface != 0 ) {                                                    \
        positional_bind( *(p).iface );                                        \
    } else if( (p).port != 0 ) {                                              \
        positional_bind( *(p).port );                                         \
    } else {                                                                  \
        return;                                                               \
    }


void
dynamic_module::operator () ( const sc_bind_proxy& p001,
			 const sc_bind_proxy& p002,
			 const sc_bind_proxy& p003,
			 const sc_bind_proxy& p004,
			 const sc_bind_proxy& p005,
			 const sc_bind_proxy& p006,
			 const sc_bind_proxy& p007,
			 const sc_bind_proxy& p008,
			 const sc_bind_proxy& p009,
			 const sc_bind_proxy& p010,
			 const sc_bind_proxy& p011,
			 const sc_bind_proxy& p012,
			 const sc_bind_proxy& p013,
			 const sc_bind_proxy& p014,
			 const sc_bind_proxy& p015,
			 const sc_bind_proxy& p016,
			 const sc_bind_proxy& p017,
			 const sc_bind_proxy& p018,
			 const sc_bind_proxy& p019,
			 const sc_bind_proxy& p020,
			 const sc_bind_proxy& p021,
			 const sc_bind_proxy& p022,
			 const sc_bind_proxy& p023,
			 const sc_bind_proxy& p024,
			 const sc_bind_proxy& p025,
			 const sc_bind_proxy& p026,
			 const sc_bind_proxy& p027,
			 const sc_bind_proxy& p028,
			 const sc_bind_proxy& p029,
			 const sc_bind_proxy& p030,
			 const sc_bind_proxy& p031,
			 const sc_bind_proxy& p032,
			 const sc_bind_proxy& p033,
			 const sc_bind_proxy& p034,
			 const sc_bind_proxy& p035,
			 const sc_bind_proxy& p036,
			 const sc_bind_proxy& p037,
			 const sc_bind_proxy& p038,
			 const sc_bind_proxy& p039,
			 const sc_bind_proxy& p040,
			 const sc_bind_proxy& p041,
			 const sc_bind_proxy& p042,
			 const sc_bind_proxy& p043,
			 const sc_bind_proxy& p044,
			 const sc_bind_proxy& p045,
			 const sc_bind_proxy& p046,
			 const sc_bind_proxy& p047,
			 const sc_bind_proxy& p048,
			 const sc_bind_proxy& p049,
			 const sc_bind_proxy& p050,
			 const sc_bind_proxy& p051,
			 const sc_bind_proxy& p052,
			 const sc_bind_proxy& p053,
			 const sc_bind_proxy& p054,
			 const sc_bind_proxy& p055,
			 const sc_bind_proxy& p056,
			 const sc_bind_proxy& p057,
			 const sc_bind_proxy& p058,
			 const sc_bind_proxy& p059,
			 const sc_bind_proxy& p060,
			 const sc_bind_proxy& p061,
			 const sc_bind_proxy& p062,
			 const sc_bind_proxy& p063,
			 const sc_bind_proxy& p064 )
{
    static bool warn_only_once=true;
    if ( m_port_index > 0 && warn_only_once )
    {
        warn_only_once = false;
	SC_REPORT_INFO(SC_ID_IEEE_1666_DEPRECATION_,
	 "multiple () binding deprecated, use explicit port binding instead." );
    }

    TRY_BIND( p001 );
    TRY_BIND( p002 );
    TRY_BIND( p003 );
    TRY_BIND( p004 );
    TRY_BIND( p005 );
    TRY_BIND( p006 );
    TRY_BIND( p007 );
    TRY_BIND( p008 );
    TRY_BIND( p009 );
    TRY_BIND( p010 );
    TRY_BIND( p011 );
    TRY_BIND( p012 );
    TRY_BIND( p013 );
    TRY_BIND( p014 );
    TRY_BIND( p015 );
    TRY_BIND( p016 );
    TRY_BIND( p017 );
    TRY_BIND( p018 );
    TRY_BIND( p019 );
    TRY_BIND( p020 );
    TRY_BIND( p021 );
    TRY_BIND( p022 );
    TRY_BIND( p023 );
    TRY_BIND( p024 );
    TRY_BIND( p025 );
    TRY_BIND( p026 );
    TRY_BIND( p027 );
    TRY_BIND( p028 );
    TRY_BIND( p029 );
    TRY_BIND( p030 );
    TRY_BIND( p031 );
    TRY_BIND( p032 );
    TRY_BIND( p033 );
    TRY_BIND( p034 );
    TRY_BIND( p035 );
    TRY_BIND( p036 );
    TRY_BIND( p037 );
    TRY_BIND( p038 );
    TRY_BIND( p039 );
    TRY_BIND( p040 );
    TRY_BIND( p041 );
    TRY_BIND( p042 );
    TRY_BIND( p043 );
    TRY_BIND( p044 );
    TRY_BIND( p045 );
    TRY_BIND( p046 );
    TRY_BIND( p047 );
    TRY_BIND( p048 );
    TRY_BIND( p049 );
    TRY_BIND( p050 );
    TRY_BIND( p051 );
    TRY_BIND( p052 );
    TRY_BIND( p053 );
    TRY_BIND( p054 );
    TRY_BIND( p055 );
    TRY_BIND( p056 );
    TRY_BIND( p057 );
    TRY_BIND( p058 );
    TRY_BIND( p059 );
    TRY_BIND( p060 );
    TRY_BIND( p061 );
    TRY_BIND( p062 );
    TRY_BIND( p063 );
    TRY_BIND( p064 );
}

} // namespace sc_core


