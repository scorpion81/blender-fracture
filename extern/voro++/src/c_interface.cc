#include "c_interface.hh"
#include "voro++.hh"

extern "C" {

	container* container_new(double ax_,double bx_,double ay_,double by_,double az_,double bz_,
							 int nx_,int ny_,int nz_,int xperiodic_,int yperiodic_,int zperiodic_,int init_mem)
	{
		
		return new voro::container(ax_, bx_, ay_, by_, az_, bz_, nx_, ny_, nz_,
								   xperiodic_, yperiodic_, zperiodic_, init_mem);
	}
	
	particle_order* particle_order_new(void)
	{
		return (particle_order*)(new voro::particle_order());
	}
	
	loop_order* loop_order_new(container* con, particle_order* p_order)
	{
		voro::container* c = (voro::container*)con;
		voro::particle_order* po = (voro::particle_order*)p_order;
		
		return (loop_order*)(new voro::c_loop_order(*c, *po));
	}
	
	void container_put(container* con, particle_order* p_order, int n,double x,double y,double z)
	{
		voro::container* c = (voro::container*)con;
		voro::particle_order* po = (voro::particle_order*)p_order;
		
		if (po)
		{
			c->put(*po, n, x, y, z);
		}
		else
		{
			c->put(n, x, y, z);
		}
		
	}
	
	void container_print_custom(loop_order* l_order, container* con, const char* format, FILE* fp)
	{
		voro::container* c = (voro::container*)con;
		voro::c_loop_order* lo = (voro::c_loop_order*)l_order;
		
		c->print_custom(*lo, format, fp);
	}

}

