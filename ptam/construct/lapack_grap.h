/* 
 * File:   lapack_grap.h
 * Author: thanh
 *
 * Created on 3 October 2009, 2:44 PM
 */

#ifndef _LAPACK_GRAP_H
#define	_LAPACK_GRAP_H

#ifdef	__cplusplus
extern "C" {
#endif

#define dgemm_ dgemm
#define dscal_ dscal
#define dsyr2k_ dsyr2k
#define drot_ drot
#define dswap_ dswap
#define dcopy_ dcopy
#define dgemv_ dgemv
#define dtrmm_ dtrmm
#define dnrm2_ dnrm2
#define dsymv_ dsymv
#define ddot_ ddot
#define daxpy_ daxpy
#define dsyr2_ dsyr2

#ifdef	__cplusplus
}
#endif

#endif	/* _LAPACK_GRAP_H */

