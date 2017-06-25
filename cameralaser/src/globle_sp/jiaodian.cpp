#include <iostream>
#include <math.h>
#include "matrix_func.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include "jiaodian.h"
#define PI 3.14159
#define NP 30
using namespace std;


int cut_part(int *x, int *y, int *X, int *Y, int &num_D, int big_part[300][3], int &parts){
	//提取有效点
	int LC[300][3] = { 0 };
	int d[300] = { 0 };
	num_D = 0;
	int D[300][2] = { 0 };
	for (int i = 0; i < 270; i++)
	{
		d[i] = sqrt(x[i] * x[i] + y[i] * y[i]);
		if (d[i] != 0){
			D[num_D][0] = i;
			D[num_D][1] = d[i];
			X[num_D] = d[i] * cos(i*PI / 180);
			Y[num_D] = d[i] * sin(i*PI / 180);
			num_D++;
		}
	}

	double threshold;
	int num_count = 0;
	int part_num = 0;
	for (int i = 0; i < num_D - 1; i++)
	{
		threshold = 0.05*D[i][1];//待定
		int dist = abs(sqrt((X[i + 1] - X[i])*(X[i + 1] - X[i]) + (Y[i + 1] - Y[i])*(Y[i + 1] - Y[i])));
		if (dist < threshold){
			num_count++;
					}
		else{
			LC[part_num][0] = num_count + 1;
			LC[part_num][1] = i - num_count;
			LC[part_num][2] = i;
			num_count = 0;
			part_num++;
		}
		if (i == num_D - 1){
			LC[part_num][0] = num_count + 1;
			LC[part_num][1] = i + 1 - num_count;
			LC[part_num][2] = i + 1;
			part_num++;
		}

	}
	parts = 0;
	for (int i = 0; i < part_num; i++)
	{
		if (LC[i][0]>4){
			big_part[parts][0] = LC[i][0];
			big_part[parts][1] = LC[i][1];
			big_part[parts][2] = LC[i][2];
			parts++;
		}
	}
	return 0;
}
/*阈值分割完成后可以在每个大类中进行特征点提取（角点或者颜色特征点）*/

//分割合并与角点提取
int get_corner_point(int*X, int *Y, int parts, int big_part[300][3], int *corner_num, int &corn_num){
	corn_num = 0;
	int d_line[300] = { 0 };
	for (int k = 0; k < parts; k++){
		int start_p[300] = { big_part[k][1] };
		int end_p[300] = { big_part[k][2] };
		int j = 0;
		while (end_p[j] != 0){
			double K = 0;
			double A = Y[start_p[j]] - Y[end_p[j]];
			double B = X[end_p[j]] - X[start_p[j]];
			double C = -1 * B*Y[end_p[j]] - A*X[end_p[j]];
			//double *d_line = new double[end_p[j] - start_p[j] + 1];
			
			double ABC = 0; double AB = 0;
			for (int i = start_p[j]; i < end_p[j] + 1; i++){
				ABC = A*X[i] + B*Y[i] + C;
				AB = sqrt(A*A + B*B);
				if (AB != 0){
					d_line[i] = abs(ABC / AB);
					//cout << "d_line"<<i<<"  "<<d_line[i] << endl;
				}
				//cout << d_line[i] <<endl;
			}
			int temp = 0;
			for (int i = start_p[j]; i <end_p[j] + 1; i++){
				if (d_line[i]>temp){
					temp = d_line[i];
				}
			}
			//cout << "temp " << temp << endl;
			if (temp > 70){//阈值待定
				for (int i = start_p[j]; i < end_p[j] + 1; i++){
					if (d_line[i]==temp){
						start_p[2 * j + 1] = start_p[j]; end_p[2 * j + 1] = i;
						start_p[2 * j + 2] = i;			 end_p[2 * j + 2] = end_p[j];
						corner_num[corn_num] = i;
						corn_num++;
						//cout << "get corn_num  " << corn_num << endl;
						break;
					}
				}
			}
			//delete[]d_line;
			j++;
		}
	}
	return 1;
}

int cal_dist(double*Px, double*Py, int num_sp, double dista[NP][NP]){
	//double dista[10][10];
	memset(dista, 0, sizeof(dista));
	for (int p = 0; p < num_sp; p++){
		int num_dl = 0;
		for (int j = 0; j < num_sp; j++){
			if (p != j){
				dista[p][num_dl] = sqrt((Px[p] - Px[j])*(Px[p] - Px[j]) + (Py[p] - Py[j])*(Py[p] - Py[j]));
				num_dl++;
			}
		}
	}
	return 0;
}

int simularfunc(int num_now, int num_last, double dist_org[NP][NP], double dist_loc[NP][NP], int *globle_id, int *loc_id){
	int simular_num = 0;
	double simular_globle[NP];

	for (int p = 0; p < num_now; p++){
		double temp = 0;
		int simular[NP];
		//~ double simular_dis_tmp[N];
		memset(simular, 0, sizeof(simular));
		for (int k = 0; k < num_last; k++){  //匹配
			for (int l = 0; l < num_now - 1; l++){
				for (int j = 0; j < num_last - 1; j++){
					if (abs(dist_org[k][j] - dist_loc[p][l]) < 20){
						//std::cout<<" simufunc distance:"<< abs(dist_org[k][j] - dist_loc[p][l]) <<endl;                        simular[k]++;
						simular[k]++;
						break;
					}
				}
			}
		}
		for (int k = 0; k < num_last; k++){ //相似度最高
			if (simular[k]>temp){
				temp = simular[k];
			}
		}
		//~ if (temp < num_lmd / 2){ //相似度不足1/2，该点不在map里  && (temp < 3)
		if (temp < num_last/2){
			//std::cout << "simular[] < 3\n" << temp << endl;
			continue;
		}
		/*for (int k = 0; k < num_last; k++){
			if (simular[k] == temp){
				globle_id[simular_num] = k;
				loc_id[simular_num] = p;
				simular_globle[simular_num] = temp;
			}
		}
		simular_num++;*/
		for (int k = 0; k < num_last; k++){
			if (simular[k] == temp){
				//避免两个点都匹配到同一个地图点
				for (int j = 0; j < simular_num; j++){
					if (globle_id[j] == k){
						//std::cout<<"match the same\n"<<endl;
						if (temp > simular_globle[j]){//匹配度更高
							loc_id[j] = p;
							simular_globle[j] = temp;
						}
						else{
							//std::cout << "same and smaller\n" << endl;
							goto next;
						}
					}
				}
				globle_id[simular_num] = k;
				loc_id[simular_num] = p;
				simular_globle[simular_num] = temp; //存下与距离匹配度最大的点的匹配值(在最小二乘时可以作为可信度加权)

				for (int t = 0; t < NP; t++){//某一个点匹配成功后将该点的距离矩阵置零
					dist_org[k][t] = 0;
				}

			}
		}

		simular_num++;
	next:
		temp = 0;//do nothing
	}
	return simular_num;
}
//计算x,y,theta
int caculate_RT(int simular_num, int *globle_id, int *loc_id, double *Px0, double *Py0, double*Px, double*Py, double*rt){

	double *p_loc = new double[(2 * simular_num) * 6];
	double *p_org = new double[2 * simular_num];
	memset(p_loc, 0, sizeof(double)* 12 * simular_num);
	memset(p_org, 0, sizeof(double)* 2 * simular_num);
	for (int p = 0; p < simular_num; p++){
		p_org[2 * p] = Px0[globle_id[p]]; p_org[2 * p + 1] = Py0[globle_id[p]];
		for (int k = 0; k < 6; k++){
			switch (k){
			case 0:
				p_loc[2 * p * 6 + k] = Px[loc_id[p]]; p_loc[(2 * p + 1) * 6 + k] = 0; break;
			case 1:
				p_loc[2 * p * 6 + k] = Py[loc_id[p]]; p_loc[(2 * p + 1) * 6 + k] = 0; break;
			case 2:
				p_loc[2 * p * 6 + k] = 1; p_loc[(2 * p + 1) * 6 + k] = 0; break;
			case 3:
				p_loc[2 * p * 6 + k] = 0; p_loc[(2 * p + 1) * 6 + k] = Px[loc_id[p]]; break;
			case 4:
				p_loc[2 * p * 6 + k] = 0; p_loc[(2 * p + 1) * 6 + k] = Py[loc_id[p]]; break;
			case 5:
				p_loc[2 * p * 6 + k] = 0; p_loc[(2 * p + 1) * 6 + k] = 1; break;
			}
		}
	}


	memset(rt, 0, sizeof(rt));
	Matrix_Solve(p_loc, p_org, 2 * simular_num, 6, rt);//simular_num不足3,no solve
	delete[]p_org;
	delete[]p_loc;
	return 0;
}

int caculate_RT1(int simular_num, int *globle_id, int *loc_id, double *Px0, double *Py0, double*Px, double*Py, double*rt){

	double *p_loc = new double[(2 * simular_num) * 4];
	double *p_org = new double[2 * simular_num];
	memset(p_loc, 0, sizeof(double)* 8 * simular_num);
	memset(p_org, 0, sizeof(double)* 2 * simular_num);
	for (int p = 0; p < simular_num; p++){
		p_org[2 * p] = Px0[globle_id[p]]; p_org[2 * p + 1] = Py0[globle_id[p]];
		for (int k = 0; k < 4; k++){
			switch (k){
			case 0:
				p_loc[2 * p * 4 + k] = Px[loc_id[p]]; p_loc[(2 * p + 1) * 4 + k] = Py[loc_id[p]]; break;
			case 1:
				p_loc[2 * p * 4 + k] = -Py[loc_id[p]]; p_loc[(2 * p + 1) * 4 + k] = Px[loc_id[p]]; break;
			case 2:
				p_loc[2 * p * 4 + k] = 1; p_loc[(2 * p + 1) * 4 + k] = 0; break;
			case 3:
				p_loc[2 * p * 4 + k] = 0; p_loc[(2 * p + 1) * 4 + k] = 1; break;
			}
		}
	}


	memset(rt, 0, sizeof(rt));
	Matrix_Solve(p_loc, p_org, 2 * simular_num, 4, rt);//simular_num不足2,no solve
	delete[]p_org;
	delete[]p_loc;
	return 0;
}
