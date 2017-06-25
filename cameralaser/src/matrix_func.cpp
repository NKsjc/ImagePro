#include <iostream>
using namespace std;

void Matrix_T(double *K, int m, int n, double *KT)//���ؾ���K��ת��KT.k[m][n]
{
	int i, j, a, b;
	for (i = 0, a = 0; i<m; i++)
	{
		for (j = 0, b = 0; j<n; j++)
		{
			KT[b + i] = K[a + j];
			b += m;
		}
		a += n;
	}
}

void Matrix_Mul(double *Mul1, int Mul1_m, double *Mul2, int Mul2_n, int nm, double *Mul)
{
	//Mul1[Mul1_m][nm]*Mul2[nm][Mul2_n]=Mul������ĳ˷�
	int i, j, k, a, b, c, d;
	for (i = 0, a = 0, c = 0; i<Mul1_m; i++)
	{
		for (j = 0; j<Mul2_n; j++)
		{
			b = a + j;
			Mul[b] = 0;
			for (k = 0, d = 0; k<nm; k++)
			{
				Mul[b] += Mul1[c + k] * Mul2[d + j];
				d += Mul2_n;
			}
		}
		c += nm;
		a += Mul2_n;
	}
}

bool Matrix_LU(double *K, int n, double *L, double *U)//�Է���K����LU�ֽ�.�ֽ�ʧ�ܷ���False.�ɹ�����True�Լ��ֽ�õ���L��U
{
	int i, j, a, b, c, d;
	double temp;
	for (i = 0, a = 0; i<n; i++)
	{
		for (j = 0; j<n; j++)
		{
			L[a + j] = U[a + j] = 0;
		}
		U[a + i] = 1;
		a += n;
	}
	for (j = 0, d = 0; j<n; j++)
	{
		for (i = j, b = d; i<n; i++)
		{
			temp = 0;
			a = 0, c = j;
			while (a<j)
			{
				temp += L[b + a] * U[c];
				c += n;
				a++;
			}
			L[b + j] = K[b + j] - temp;
			b += n;
		}
		i = j + 1;
		while (i<n)
		{
			temp = 0;
			a = 0, c = i;
			while (a<j)
			{
				temp += L[d + a] * U[c];
				a++;
				c += n;
			}
			if (L[d + j] == 0)
			{
				return false;
			}
			U[d + i] = (K[d + i] - temp) / L[d + j];
			i++;
		}
		d += n;
	}
	return true;
}

bool Matrix_Inv(double *K, int n, double *InvK)//����LU�ֽⷽ������K����InvK,K[n][n]
{
	if (1 == n)
	{
		if (K[0] == 0)
		{
			return false;
		}
		else
		{
			InvK[0] = 1 / K[0];
		}
	}
	else if (n<1)
	{
		return false;
	}
	else
	{
		int i, j, a, b;
		double *L, *U, *d, *x, *e, temp;
		a = n*n;
		L = new double[a];
		U = new double[a];
		if (Matrix_LU(K, n, L, U))
		{
			d = new double[n];
			x = new double[n];
			e = new double[n];
			for (i = 0; i<n; i++)
			{
				x[i] = d[i] = 0;
			}
			for (i = 0; i<n; i++)
			{
				for (j = 0; j<n; j++)
				{
					e[j] = 0;
				}
				e[i] = 1;
				j = 0;
				b = 0;
				while (j<n)
				{
					temp = 0;
					a = 0;
					while (a<j)
					{
						temp += d[a] * L[b + a];
						a++;
					}
					d[j] = e[j] - temp;
					d[j] /= L[b + j];
					j++;
					b += n;
				}
				j = n - 1;
				b -= n;
				while (j>-1)
				{
					temp = 0;
					a = j + 1;
					while (a<n)
					{
						temp += U[b + a] * x[a];
						a++;
					}
					x[j] = d[j] - temp;
					x[j] /= U[b + j];
					j--;
					b -= n;
				}
				for (j = 0, b = i; j<n; j++)
				{
					InvK[b] = x[j];
					b += n;
				}
			}
			delete[]d;
			delete[]x;
			delete[]e;
		}
		else
		{
			delete[]L;
			delete[]U;
			return false;
		}
		delete[]L;
		delete[]U;
	}
	return true;
}

bool Matrix_Solve(double *K, double *B, int m, int n, double *x)//Kx=B���x��K[m][n]������������С���˽�,B[m][1]
{
	double *KT, *Kmul, *Kb, *Kinv;
	int i;
	i = n*n;
	KT = new double[m*n];
	Kmul = new double[i];
	Kinv = new double[i];
	Kb = new double[n];
	Matrix_T(K, m, n, KT);
	Matrix_Mul(KT, n, K, n, m, Kmul);
	Matrix_Mul(KT, n, B, 1, m, Kb);
	if (Matrix_Inv(Kmul, n, Kinv))
	{
		Matrix_Mul(Kinv, n, Kb, 1, n, x);
		delete[]KT;
		delete[]Kmul;
		delete[]Kinv;
		delete[]Kb;
		return true;
	}
	else
	{
		delete[]KT;
		delete[]Kmul;
		delete[]Kinv;
		delete[]Kb;
		return false;
	}
}
