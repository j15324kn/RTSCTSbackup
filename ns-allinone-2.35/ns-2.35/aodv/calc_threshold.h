#ifndef __CALC_THRESHOLD_H__
#define __CALC_THRESHOLD_H__ 1
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
    double rev_threshold(double rxThresh_);
    double Friis(double Pt,double Gt, double Gr, double lambda, double L, double rxThresh_);
    double TwoRay(double Pt, double Gt, double Gr, double ht, double hr,double L, double rxThresh_);
#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* __CALC_THRESHOLD_H__*/
