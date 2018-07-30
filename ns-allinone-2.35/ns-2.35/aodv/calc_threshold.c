/*-----------------------------------------------------------------------------
	電波強度から距離を求めるプログラム
	TwoRayGroundモデルでのみ有効
	他のaodvディレクトリ内のソースからの参照に対応するため、一部の処理を変更している
	作成日　2016/12/18
-----------------------------------------------------------------------------*/
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include "calc_threshold.h"

#ifndef M_PI
#define M_PI 3.14159265359
#endif
double Friis(double Pt,double Gt, double Gr, double lambda, double L, double rxThresh_)
{
	return lambda / ( 4 * M_PI * sqrt( rxThresh_ / Pt ) );
}

double TwoRay(double Pt, double Gt, double Gr, double ht, double hr,double L, double rxThresh_)
{
	return sqrt(sqrt(Pt*Gt*Gr*ht*ht*hr*hr/(L*rxThresh_)));
}

//int main(int argc,char *argv[])
double rev_threshold(double rxThresh_)
{
	//初期値の設定
	double Pt =  0.28183815;		// transmit power 送信電力
	double Gt = 1.0;				// transmit antenna gain 送信アンテナ利得
	double Gr = 1.0;				// receive antenna 受信アンテナ利得
	double freq = 914.0e6;			// frequency 周波数
	double sysLoss = 1.0;			// system loss 
	
	// for two-ray model
	double ht = 1.5;				// transmit antenna height 送信アンテナ高度
	double hr = 1.5;				// receive antenna height
	//double rxThresh_=0;				//receiving threshold
	double dist;					//distance
	
	// (3*10^8) / (914*10^6) = 0.328227571116 ?
	double lambda = 3.0e8/freq;
	
	//引数の解析
	//xThresh_ = atof(argv[1]);
	
	//crossover_dist これ以上ならfree spaceでの計算
	//86.1424705614252m ≒　2.59117e-08
	if(rxThresh_ > 2.59117e-08){
		//free spaceでの計算
		//printf("Caliculate of Friis\n");
		dist=Friis(Pt,Gt,Gr,lambda,sysLoss,rxThresh_);
	}else{
		//two ray groundでの計算
		//printf("Caliculate of TwoRay\n");
		dist=TwoRay(Pt,Gr,Gr,ht,hr,sysLoss,rxThresh_);
	}
	/*double crossover_dist = (4 * M_PI * ht * hr) / lambda;
	printf("crossover_dist : %0.10f\n",crossover_dist);
	if(rxThresh_ < crossover_dist){
		//free spaceでの計算
		printf("Caliculate of Friis\n");
		dist=Friis(Pt,Gt,Gr,lambda,sysLoss,rxThresh_);
	}else{
		//two ray groundでの計算
		printf("Caliculate of TwoRay\n");
		dist=TwoRay(Pt,Gr,Gr,ht,hr,sysLoss,rxThresh_);
	}*/
	return dist;
}
