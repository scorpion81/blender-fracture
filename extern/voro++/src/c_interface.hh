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

container* container_new(double ax_,double bx_,double ay_,double by_,double az_,double bz_,
                         int nx_,int ny_,int nz_,int xperiodic_,int yperiodic_,int zperiodic_,int init_mem);
particle_order* particle_order_new(void);
loop_order* loop_order_new(container* con, particle_order* po);

void container_put(container* con, particle_order* po, int n, double x, double y, double z);
void container_print_custom(loop_order* lo, container* con, const char* format, FILE* fp);

#ifdef __cplusplus
}
#endif

#endif /* VORO_C_INTERFACE_H */
