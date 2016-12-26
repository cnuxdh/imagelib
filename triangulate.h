/* 
 *  Copyright (c) 2008  Noah Snavely (snavely (at) cs.washington.edu)
 *    and the University of Washington
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 */

/* triangulate.c */
/* Triangulate two image points */

#ifndef __triangulate_h__
#define __triangulate_h__

#ifdef __cplusplus
extern "C" {
#endif

#include "vector.h"

/* Project a point onto an image */
v2_t project(double *R, double *t0, double *P);

/* Find the point with the smallest squared projection error */
v3_t triangulate(v2_t p, v2_t q, 
		 double *R0, double *t0, 
		 double *R1, double *t1, double *error);

/* Find the point with the smallest squared projection error */
v3_t triangulate_n(int num_points, 
		   v2_t *p, double *R, double *t, double *error_out);

/* Find the point with the smallest squared projection error */
v3_t triangulate_n_refine(v3_t pt, int num_points, 
			  v2_t *p, double *R, double *t, double *error_out);

/* Given an F matrix, two calibration matrices, and a point 
 * correspondence, find R and t */
void find_extrinsics(double *F, double *K1, double *K2, 
		     v2_t p1, v2_t p2, double *R, double *t);

/* Given an E matrix and a point correspondence, find R and t */
int find_extrinsics_essential(double *E, v2_t p1, v2_t p2, 
                              double *R, double *t);


/* for pano: Given an E matrix and a point correspondence, find R and t */
int find_extrinsics_essential_multipt_pano(double *E, int n,
										   v3_t *p1, v3_t *p2, 
										   double *R, double *t);


int find_extrinsics_essential_multipt(double *E, int n,
                                      v2_t *p1, v2_t *p2, 
                                      double *R, double *t);


/* Solve for a 3x4 projection matrix, given a set of 3D points and 2D
 * projections */
int find_projection_3x4(int num_pts, v3_t *points, v2_t *projs, double *P);

/* Solve for a 3x4 projection matrix using RANSAC, given a set of 3D
 * points and 2D projections */
int find_projection_3x4_ransac(int num_pts, v3_t *points, v2_t *projs, 
			       double *P,
			       int ransac_rounds, double ransac_threshold);

//for panorama
int find_projection_3x4_pano(int num_pts, v3_t *points, v3_t *projs, double *P);

int find_projection_3x4_ransac_pano(int num_pts, v3_t *points, v3_t *projs, 
	double *P, int ransac_rounds, double ransac_threshold);

double cross_angle(v3_t p1, v3_t p2);

#ifdef __cplusplus
}
#endif

#endif /* __triangulate_h__ */
