#include <iostream>
#include "matrix_func.h"
#include "test1function.h"
//#define  PI  3.14159
#define NP 30

int cut_part(int *x, int *y, int *X, int *Y, int &num_D, int big_part[300][3], int &parts);
int get_corner_point(int*X, int *Y, int parts, int big_part[300][3], int *corner_num, int &corn_num);
int cal_dist(double*Px, double*Py, int num_sp, double dista[NP][NP]);
int simularfunc(int num_now, int num_last, double dist_org[NP][NP], double dist_loc[NP][NP], int *globle_id, int *loc_id);
int caculate_RT(int simular_num, int *globle_id, int *loc_id, double *Px0, double *Py0, double*Px, double*Py, double*rt);
int caculate_RT1(int simular_num, int *globle_id, int *loc_id, double *Px0, double *Py0, double*Px, double*Py, double*rt);

