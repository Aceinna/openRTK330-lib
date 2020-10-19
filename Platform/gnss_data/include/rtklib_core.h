#ifndef _RTKLIB_CORE_H_
#define _RTKLIB_CORE_H_

#ifdef __cplusplus
extern "C" {
#endif

#if !defined(WIN32) && defined(_WINDOWS)
#define WIN32
#endif

#include <time.h>

#define ENAGLO
#define ENACMP
#define ENAGAL

#define NFREQ       2
#ifndef NFREQ
#define NFREQ       3                   /* number of carrier frequencies */
#endif
#define NFREQGLO    2                   /* number of carrier frequencies of GLONASS */

#ifndef NEXOBS
#define NEXOBS      0                   /* number of extended obs codes */
#endif

#define STRFMT_RTCM2 0                  /* stream format: RTCM 2 */
#define STRFMT_RTCM3 1                  /* stream format: RTCM 3 */
#define STRFMT_OEM4  2                  /* stream format: NovAtel OEMV/4 */
#define STRFMT_CNAV  3                  /* stream format: ComNav */
#define STRFMT_UBX   4                  /* stream format: u-blox LEA-*T */
#define STRFMT_SBP   5                  /* stream format: Swift Navigation SBP */
#define STRFMT_CRES  6                  /* stream format: Hemisphere */
#define STRFMT_STQ   7                  /* stream format: SkyTraq S1315F */
#define STRFMT_GW10  8                  /* stream format: Furuno GW10 */
#define STRFMT_JAVAD 9                  /* stream format: JAVAD GRIL/GREIS */
#define STRFMT_NVS   10                 /* stream format: NVS NVC08C */
#define STRFMT_BINEX 11                 /* stream format: BINEX */
#define STRFMT_RT17  12                 /* stream format: Trimble RT17 */
#define STRFMT_SEPT  13                 /* stream format: Septentrio */
#define STRFMT_CMR   14                 /* stream format: CMR/CMR+ */
#define STRFMT_TERSUS 15                /* stream format: TERSUS */
#define STRFMT_LEXR  16                 /* stream format: Furuno LPY-10000 */
#define STRFMT_RINEX 17                 /* stream format: RINEX */
#define STRFMT_SP3   18                 /* stream format: SP3 */
#define STRFMT_RNXCLK 19                /* stream format: RINEX CLK */
#define STRFMT_SBAS  20                 /* stream format: SBAS messages */
#define STRFMT_NMEA  21                 /* stream format: NMEA 0183 */

#ifdef WIN32
typedef struct {                      /* time struct */
	time_t time;                        /* time (s) expressed by standard time_t */
	double sec;                         /* fraction of second under 1 s */
} gtime_t;
#else
typedef struct {                            /* time struct */
	time_t time;                            /* time (s) expressed by standard time_t */
	__attribute__((aligned(8)))double sec;  /* fraction of second under 1 s */
} gtime_t;
#endif /*WIN32*/

typedef struct {                      /* observation data record */
	gtime_t time;                       /* receiver sampling time (GPST) */
	//gtime_t eventime;                 /* time of event (GPST) */
	int timevalid;                      /* time is valid (Valid GNSS fix) for time mark */
	unsigned char sat, rcv;             /* satellite/receiver number */
	unsigned char sys, prn;
	unsigned char SNR[NFREQ + NEXOBS];  /* signal strength (0.25 dBHz) */
	unsigned char LLI[NFREQ + NEXOBS];  /* loss of lock indicator */
	unsigned char code[NFREQ + NEXOBS]; /* code indicator (CODE_???) */
	//unsigned char qualL[NFREQ + NEXOBS]; /* quality of carrier phase measurement */
	//unsigned char qualP[NFREQ + NEXOBS]; /* quality of pseudorange measurement */
	//unsigned char freq; /* GLONASS frequency channel (0-13) */
	double L[NFREQ + NEXOBS];           /* observation data carrier-phase (cycle) */
	double P[NFREQ + NEXOBS];           /* observation data pseudorange (m) */
	float  D[NFREQ + NEXOBS];           /* observation data doppler frequency (Hz) */
	float azel[2];
} obsd_t;

typedef struct {                      /* GPS/QZS/GAL broadcast ephemeris type */
	int sat;                            /* satellite number */
	int iode, iodc;                     /* IODE,IODC */
	int sva;                            /* SV accuracy (URA index) */
	int svh;                            /* SV health (0:ok) */
	int week;                           /* GPS/QZS: gps week, GAL: galileo week */
	int code;                           /* GPS/QZS: code on L2 */
					              	    /* GAL: data source defined as rinex 3.03 */
						                /* BDS: data source (0:unknown,1:B1I,2:B1Q,3:B2I,4:B2Q,5:B3I,6:B3Q) */
	int flag;                           /* GPS/QZS: L2 P data flag */
						                /* BDS: nav type (0:unknown,1:IGSO/MEO,2:GEO) */
	gtime_t toe, toc, ttr;              /* Toe,Toc,T_trans */
						                /* SV orbit parameters */
	double A, e, i0, OMG0, omg, M0, deln, OMGd, idot;
	double crc, crs, cuc, cus, cic, cis;
	double toes;                        /* Toe (s) in week */
	double fit;                         /* fit interval (h) */
	double f0, f1, f2;                  /* SV clock parameters (af0,af1,af2) */
	double tgd[4];                      /* group delay parameters */
						                /* GPS/QZS:tgd[0]=TGD */
						                /* GAL    :tgd[0]=BGD E5a/E1,tgd[1]=BGD E5b/E1 */
						                /* CMP    :tgd[0]=BGD1,tgd[1]=BGD2 */
	double Adot, ndot;                  /* Adot,ndot for CNAV */
} eph_t;

typedef struct {                      /* GLONASS broadcast ephemeris type */
	int sat;                            /* satellite number */
	int iode;                           /* IODE (0-6 bit of tb field) */
	int frq;                            /* satellite frequency number */
	int svh, sva, age;                  /* satellite health, accuracy, age of operation */
	gtime_t toe;                        /* epoch of epherides (gpst) */
	gtime_t tof;                        /* message frame time (gpst) */
	double pos[3];                      /* satellite position (ecef) (m) */
	double vel[3];                      /* satellite velocity (ecef) (m/s) */
	double acc[3];                      /* satellite acceleration (ecef) (m/s^2) */
	double taun, gamn;                  /* SV clock bias (s)/relative freq bias */
	double dtaun;                       /* delay between L1 and L2 (s) */
} geph_t;

#define _OPENRTK_

#ifdef __cplusplus
}
#endif

#endif
