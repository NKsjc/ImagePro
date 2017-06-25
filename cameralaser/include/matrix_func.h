#include <iostream>
void Matrix_T(double *K, int m, int n, double *KT);
void Matrix_Mul(double *Mul1, int Mul1_m, double *Mul2, int Mul2_n, int nm, double *Mul);
bool Matrix_LU(double *K, int n, double *L, double *U);
bool Matrix_Inv(double *K, int n, double *InvK);
bool Matrix_Solve(double *K, double *B, int m, int n, double *x);