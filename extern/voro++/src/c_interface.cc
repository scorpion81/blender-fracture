#include <stdio.h>

#include "c_interface.hh"
#include "voro++.hh"

container* container_new(double ax_,double bx_,double ay_,double by_,double az_,double bz_,
                         int nx_,int ny_,int nz_,int xperiodic_,int yperiodic_,int zperiodic_,int init_mem)
{
	
	return (container*)(new voro::container(ax_, bx_, ay_, by_, az_, bz_, nx_, ny_, nz_,
	                                        xperiodic_, yperiodic_, zperiodic_, init_mem));
}

particle_order* particle_order_new(void)
{
	return (particle_order*)(new voro::particle_order());
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

void container_compute_cells(container* con, cell* cells)
{
	int i = 0, v = 0, fo = 0, fv = 0, n = 0;
	voro::container* cn = (voro::container*)con;
	voro::voronoicell_neighbor vc;
	voro::c_loop_all vl(*cn);
	cell c;
	if(vl.start()) {
		do {
			if (cn->compute_cell(vc,vl)) {

				// adapted from voro++
				std::vector<double> verts;
				std::vector<int> face_orders;
				std::vector<int> face_verts;
				std::vector<int> neighbors;
				double *pp, centroid[3];
				pp = vl.p[vl.ijk]+vl.ps*vl.q;

				// cell particle index
				c.index = cn->id[vl.ijk][vl.q];

				// verts
				vc.vertices(*pp, pp[1], pp[2], verts);
				c.totvert = vc.p;
				c.verts = new float[c.totvert][3];
				for (v = 0; v < c.totvert; v++) {
					c.verts[v][0] = (float)verts[v * 3];
					c.verts[v][1] = (float)verts[v * 3 + 1];
					c.verts[v][2] = (float)verts[v * 3 + 2];
				}

				// faces
				c.totpoly = vc.number_of_faces();
				vc.face_orders(face_orders);
				c.poly_totvert = new int[c.totpoly];

				for (fo = 0; fo < c.totpoly; fo++) {
					c.poly_totvert[fo] = face_orders[fo];
				}

				vc.face_vertices(face_verts);
				c.poly_indices = new int*[c.totpoly];
				int skip = 0;
				for (fo = 0; fo < c.totpoly; fo++) {
					int num_verts = c.poly_totvert[fo];
					c.poly_indices[fo] = new int[num_verts];
					for (fv = 0; fv < num_verts; fv++) {
						c.poly_indices[fo][fv] = face_verts[skip + 1 + fv];
					}
					skip += (num_verts+1);
				}

				// neighbors
				vc.neighbors(neighbors);
				c.neighbors = new int[c.totpoly];
				for (n = 0; n < c.totpoly; n++)
				{
					c.neighbors[n] = neighbors[n];
				}

				// centroid
				vc.centroid(centroid[0], centroid[1], centroid[2]);
				c.centroid[0] = (float)centroid[0] + (float)*pp;
				c.centroid[1] = (float)centroid[1] + (float)pp[1];
				c.centroid[2] = (float)centroid[2] + (float)pp[2];

				// volume
				c.volume = (float)vc.volume();

				// valid cell, store
				cells[i] = c;

			}
			else { // invalid cell, set NULL XXX TODO (Somehow !!!)
				c.centroid[0] = 0.0f;
				c.centroid[1] = 0.0f;
				c.centroid[2] = 0.0f;
				c.index = 0;
				c.neighbors = NULL;
				c.totpoly = 0;
				c.totvert = 0;
				c.poly_totvert = NULL;
				c.poly_indices = NULL;
				c.verts = NULL;
				cells[i] = c;
			}
			i++;
		}
		while(vl.inc());
	}
}

void container_free(container* con)
{
	voro::container* c = (voro::container*)con;
	delete c;
}

void particle_order_free(particle_order* p_order)
{
	voro::particle_order* po = (voro::particle_order*)p_order;
	delete po;
}

cell* cells_new(int totcells)
{
	int i = 0;
	cell c;
	cell *cl;

	//need to initalize properly in case we dont compute....
	c.centroid[0] = 0.0f;
	c.centroid[1] = 0.0f;
	c.centroid[2] = 0.0f;
	c.index = 0;
	c.neighbors = NULL;
	c.totpoly = 0;
	c.totvert = 0;
	c.poly_totvert = NULL;
	c.poly_indices = NULL;
	c.verts = NULL;

	cl = new cell[totcells];
	for (i = 0; i < totcells; i++)
	{
		cl[i] = c;
	}

	return cl;
}

void cells_free(cell *cells, int totcells)
{
	// XXX TODO free properly !
	if (cells) {
		int i = 0, j = 0;
		for (i = 0; i < totcells; i++) {
			cell c = cells[i];
			if (c.verts)
			{
				delete [] c.verts;
			}
			if (c.neighbors) delete [] c.neighbors;
			if (c.poly_indices)
			{
				for (j = 0; j < c.totpoly; j++)
				{
					delete [] c.poly_indices[j];
				}
				delete [] c.poly_indices;
			}
			if (c.poly_totvert) delete [] c.poly_totvert;
		}
		delete [] cells;
	}

}


