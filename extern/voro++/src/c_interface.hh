#ifndef VORO_C_INTERFACE_H
#define VORO_C_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif

typedef void container;
typedef void loop_order;

typedef struct particle_order {
	int *o;
	int *op;
	int size;
} particle_order;

/* Necessary Voro++ data for fracture:
 * %i the particle/cell index
 *
 * %w number of vertices (totvert)
 * %P global vertex coordinates
 * v  vertex section delimiter
 *
 * %s number of faces (totpoly)
 * %a number of vertices in each face (sum is totloop)
 * %t the indices to the cell vertices, describes which vertices build each face
 * %n neighboring cell index for each face
 * f  face section delimiter
 *
 * %C the centroid of the voronoi cell
 * c  centroid section delimiter
 */

typedef struct cell {
	float (*verts)[3];
	int *poly_totvert;
	int **poly_indices;
	int *neighbors;

	float centroid[3];
	float volume;
	int index;
	int totvert;
	int totpoly;
} cell;

container* container_new(double ax_,double bx_,double ay_,double by_,double az_,double bz_,
                         int nx_,int ny_,int nz_,int xperiodic_,int yperiodic_,int zperiodic_,int init_mem);
particle_order* particle_order_new(void);

void container_put(container* con, particle_order* po, int n, double x, double y, double z);

void container_free(container* con);
void particle_order_free(particle_order* po);

// cell array for direct access
cell* cells_new(int totcell);
void cells_free(cell* cells, int totcells);
void container_compute_cells(container* con, cell* cells);

#ifdef __cplusplus
}
#endif

#endif /* VORO_C_INTERFACE_H */
