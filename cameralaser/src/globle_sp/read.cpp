#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
using namespace std;
int main(int argc,char ** argv){
	   FILE *fp0;
               fp0=fopen("globle_sp/globle_xy.txt","r");
	
	  	int sp_num = 0;
	double Px0[10] = { 0 }, Py0[10] = { 0 };
	 char buf[1024]; 
	 while(fgets(buf,1024,fp0) != NULL){
		 sp_num++;
		 }
		 			 printf("sp_num  %d \n",sp_num);
		 			 fclose(fp0);			 
		 fp0=fopen("globle_xy.txt","r");		 
		 for(int i=0;i<sp_num;i++){
			 fscanf(fp0,"%lf  %lf",&Px0[i],&Py0[i]);
			 printf("Px0 %lf  Py0 %lf \n",Px0[i],Py0[i]);
			 }
			 fclose(fp0);
		


	
	
	
}
