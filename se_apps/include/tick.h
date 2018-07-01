/*

 Copyright (c) 2011-2013 Gerhard Reitmayr, TU Graz

 Copyright (c) 2014 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#include <sys/time.h>       /* clock_t, clock, CLOCKS_PER_SEC */

inline float tick() {

	static struct timeval t;

	struct timeval diff = t;
	gettimeofday(&t, NULL);

	return ((t.tv_sec - diff.tv_sec) * 1000000u + t.tv_usec - diff.tv_usec)
			/ 1.e6;
	/*
	 static  clock_t t = clock();
	 clock_t diff = clock() - t ;
	 t = clock();
	 return ((float)diff) / (float) CLOCKS_PER_SEC;
	 */
}

