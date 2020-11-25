
#include <time.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>

#include "rtcm.h"
#include "gnss_data_api.h"
#include "nav_math.h"
#include "uart.h"
#include "tcp_driver.h"

#define SC2RAD 3.1415926535898 /* semi-circle to radian (IS-GPS) */
#define AU 149597870691.0      /* 1 AU (m) */
#define AS2R (D2R / 3600.0)    /* arc sec to radian */
// #define CCMRAM __attribute__((section(".ccmram")))
#define RTCM2PREAMB 0x66 /* rtcm ver.2 frame preamble */
#define RTCM3PREAMB 0xD3 /* rtcm ver.3 frame preamble */

static int decode_type4001(rtcm_t *rtcm);
extern void OS_Delay(uint32_t msec);

/* set the default week numner for real-time system without a UTC time */
static uint16_t default_week_number = 2068;
static uint8_t dayofweek = 3;

static int default_glo_frq_table[30] = {
	1, -4, 05, 06, 01, -4, 05, 06, -2, -7, 00, -1, -2, -7, 00, -1, 
	04, -3, 03, 02, 04, -3, 03, 02, 0, -5, -99, -99, -99, -99
};

const static double gpst0[] = { 1980, 1, 6, 0, 0, 0 }; /* gps time reference */
//const static double gst0 []={1999,8,22,0,0,0}; /* galileo system time reference */
const static double bdt0[] = { 2006, 1, 1, 0, 0, 0 }; /* beidou time reference */

static char *obscodes[] = {
	/* observation code strings */

	"", "1C", "1P", "1W", "1Y", "1M", "1N", "1S", "1L", "1E",   /*  0- 9 */
	"1A", "1B", "1X", "1Z", "2C", "2D", "2S", "2L", "2X", "2P", /* 10-19 */
	"2W", "2Y", "2M", "2N", "5I", "5Q", "5X", "7I", "7Q", "7X", /* 20-29 */
	"6A", "6B", "6C", "6X", "6Z", "6S", "6L", "8L", "8Q", "8X", /* 30-39 */
	"2I", "2Q", "6I", "6Q", "3I", "3Q", "3X", "1I", "1Q", "5A", /* 40-49 */
	"5B", "5C", "9A", "9B", "9C", "9X", "", "", "", ""          /* 50-59 */
};
/* GPS  */
static unsigned char obsfreqs_gps[] = {
	/* 1:L1, 2:L2, 3:L5 */
	0, 1, 1, 1, 1, 1, 1, 1, 1, 1, /*  0- 9 */
	0, 0, 1, 1, 2, 2, 2, 2, 2, 2, /* 10-19 */
	2, 2, 2, 2, 3, 3, 3, 0, 0, 0, /* 20-29 */
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, /* 30-39 */
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, /* 40-49 */
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0  /* 50-59 */
};
/* GLO */
static unsigned char obsfreqs_glo[] = {
	/* 1:G1, 2:G2, 3:G3 */
	0, 1, 1, 0, 0, 0, 0, 0, 0, 0, /*  0- 9 */
	0, 0, 0, 0, 2, 0, 0, 0, 0, 2, /* 10-19 */
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, /* 20-29 */
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, /* 30-39 */
	0, 0, 0, 0, 3, 3, 3, 0, 0, 0, /* 40-49 */
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0  /* 50-59 */
};
/* GAL */
static unsigned char obsfreqs_gal[] = {
	/* 1:E1, 2:E5b, 3:E5a, 4:E5(a+b), 5:E6 */
	0, 1, 0, 0, 0, 0, 0, 0, 0, 0, /*  0- 9 */
	1, 1, 1, 1, 0, 0, 0, 0, 0, 0, /* 10-19 */
	0, 0, 0, 0, 3, 3, 3, 2, 2, 2, /* 20-29 */
	5, 5, 5, 5, 5, 0, 0, 4, 4, 4, /* 30-39 */
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, /* 40-49 */
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0  /* 50-59 */
};
/* QZSS */
static unsigned char obsfreqs_qzs[] = {
	/* 1:L1, 2:L2, 3:L5, 4:LEX, 5:S */
	0, 1, 0, 0, 0, 0, 0, 1, 1, 0, /*  0- 9 */
	0, 0, 1, 1, 0, 0, 2, 2, 2, 0, /* 10-19 */
	0, 0, 0, 0, 3, 3, 3, 0, 0, 0, /* 20-29 */
	0, 0, 0, 4, 0, 4, 4, 0, 0, 0, /* 30-39 */
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, /* 40-49 */
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0  /* 50-59 */
};
/* SBS */
static unsigned char obsfreqs_sbs[] = {
	/* 1:L1, 2:L5 */
	0, 1, 0, 0, 0, 0, 0, 0, 0, 0, /*  0- 9 */
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, /* 10-19 */
	0, 0, 0, 0, 2, 2, 2, 0, 0, 0, /* 20-29 */
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, /* 30-39 */
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, /* 40-49 */
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0  /* 50-59 */
};
/* BDS */
static unsigned char obsfreqs_cmp[] = {
	/* 1:B1, 2:B3, 3:B2 */
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, /*  0- 9 */
	0, 0, 0, 0, 0, 0, 0, 0, 1, 0, /* 10-19 */
	0, 0, 0, 0, 0, 0, 0, 3, 3, 3, /* 20-29 */
	0, 0, 0, 2, 0, 0, 0, 0, 0, 0, /* 30-39 */
	1, 1, 2, 2, 0, 0, 0, 1, 1, 0, /* 40-49 */
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0  /* 50-59 */
};
/* IRN */
static unsigned char obsfreqs_irn[] = {
	/* 1:L5, 2:S */
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, /*  0- 9 */
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, /* 10-19 */
	0, 0, 0, 0, 0, 0, 1, 0, 0, 0, /* 20-29 */
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, /* 30-39 */
	0, 0, 0, 0, 0, 0, 0, 0, 0, 1, /* 40-49 */
	1, 1, 2, 2, 2, 2, 0, 0, 0, 0  /* 50-59 */
};

static const unsigned int tbl_CRC24Q[] = {
	0x000000, 0x864CFB, 0x8AD50D, 0x0C99F6, 0x93E6E1, 0x15AA1A, 0x1933EC, 0x9F7F17,
	0xA18139, 0x27CDC2, 0x2B5434, 0xAD18CF, 0x3267D8, 0xB42B23, 0xB8B2D5, 0x3EFE2E,
	0xC54E89, 0x430272, 0x4F9B84, 0xC9D77F, 0x56A868, 0xD0E493, 0xDC7D65, 0x5A319E,
	0x64CFB0, 0xE2834B, 0xEE1ABD, 0x685646, 0xF72951, 0x7165AA, 0x7DFC5C, 0xFBB0A7,
	0x0CD1E9, 0x8A9D12, 0x8604E4, 0x00481F, 0x9F3708, 0x197BF3, 0x15E205, 0x93AEFE,
	0xAD50D0, 0x2B1C2B, 0x2785DD, 0xA1C926, 0x3EB631, 0xB8FACA, 0xB4633C, 0x322FC7,
	0xC99F60, 0x4FD39B, 0x434A6D, 0xC50696, 0x5A7981, 0xDC357A, 0xD0AC8C, 0x56E077,
	0x681E59, 0xEE52A2, 0xE2CB54, 0x6487AF, 0xFBF8B8, 0x7DB443, 0x712DB5, 0xF7614E,
	0x19A3D2, 0x9FEF29, 0x9376DF, 0x153A24, 0x8A4533, 0x0C09C8, 0x00903E, 0x86DCC5,
	0xB822EB, 0x3E6E10, 0x32F7E6, 0xB4BB1D, 0x2BC40A, 0xAD88F1, 0xA11107, 0x275DFC,
	0xDCED5B, 0x5AA1A0, 0x563856, 0xD074AD, 0x4F0BBA, 0xC94741, 0xC5DEB7, 0x43924C,
	0x7D6C62, 0xFB2099, 0xF7B96F, 0x71F594, 0xEE8A83, 0x68C678, 0x645F8E, 0xE21375,
	0x15723B, 0x933EC0, 0x9FA736, 0x19EBCD, 0x8694DA, 0x00D821, 0x0C41D7, 0x8A0D2C,
	0xB4F302, 0x32BFF9, 0x3E260F, 0xB86AF4, 0x2715E3, 0xA15918, 0xADC0EE, 0x2B8C15,
	0xD03CB2, 0x567049, 0x5AE9BF, 0xDCA544, 0x43DA53, 0xC596A8, 0xC90F5E, 0x4F43A5,
	0x71BD8B, 0xF7F170, 0xFB6886, 0x7D247D, 0xE25B6A, 0x641791, 0x688E67, 0xEEC29C,
	0x3347A4, 0xB50B5F, 0xB992A9, 0x3FDE52, 0xA0A145, 0x26EDBE, 0x2A7448, 0xAC38B3,
	0x92C69D, 0x148A66, 0x181390, 0x9E5F6B, 0x01207C, 0x876C87, 0x8BF571, 0x0DB98A,
	0xF6092D, 0x7045D6, 0x7CDC20, 0xFA90DB, 0x65EFCC, 0xE3A337, 0xEF3AC1, 0x69763A,
	0x578814, 0xD1C4EF, 0xDD5D19, 0x5B11E2, 0xC46EF5, 0x42220E, 0x4EBBF8, 0xC8F703,
	0x3F964D, 0xB9DAB6, 0xB54340, 0x330FBB, 0xAC70AC, 0x2A3C57, 0x26A5A1, 0xA0E95A,
	0x9E1774, 0x185B8F, 0x14C279, 0x928E82, 0x0DF195, 0x8BBD6E, 0x872498, 0x016863,
	0xFAD8C4, 0x7C943F, 0x700DC9, 0xF64132, 0x693E25, 0xEF72DE, 0xE3EB28, 0x65A7D3,
	0x5B59FD, 0xDD1506, 0xD18CF0, 0x57C00B, 0xC8BF1C, 0x4EF3E7, 0x426A11, 0xC426EA,
	0x2AE476, 0xACA88D, 0xA0317B, 0x267D80, 0xB90297, 0x3F4E6C, 0x33D79A, 0xB59B61,
	0x8B654F, 0x0D29B4, 0x01B042, 0x87FCB9, 0x1883AE, 0x9ECF55, 0x9256A3, 0x141A58,
	0xEFAAFF, 0x69E604, 0x657FF2, 0xE33309, 0x7C4C1E, 0xFA00E5, 0xF69913, 0x70D5E8,
	0x4E2BC6, 0xC8673D, 0xC4FECB, 0x42B230, 0xDDCD27, 0x5B81DC, 0x57182A, 0xD154D1,
	0x26359F, 0xA07964, 0xACE092, 0x2AAC69, 0xB5D37E, 0x339F85, 0x3F0673, 0xB94A88,
	0x87B4A6, 0x01F85D, 0x0D61AB, 0x8B2D50, 0x145247, 0x921EBC, 0x9E874A, 0x18CBB1,
	0xE37B16, 0x6537ED, 0x69AE1B, 0xEFE2E0, 0x709DF7, 0xF6D10C, 0xFA48FA, 0x7C0401,
	0x42FA2F, 0xC4B6D4, 0xC82F22, 0x4E63D9, 0xD11CCE, 0x575035, 0x5BC9C3, 0xDD8538 
};

static char codepris[7][MAXFREQ][16] = {
	/* code priority table */

	{"CPYWMNSL", "PYWCMNDSLX", "IQX", "", "", "", ""}, /* GPS 1:L1, 2:L2, 3:L5 */
	{"PC", "PC", "IQX", "", "", "", ""},               /* GLO 1:G1, 2:G2, 3:G3 */
	{"ABXZ", "IQX", "IQX", "IQX", "ABCXZ", "", ""},    /* GAL 1:E1, 2:E5b, 3:E5a, 4:E5(a+b), 5:E6 */
	{"CSLXZ", "SLX", "IQX", "SLX", "", "", ""},        /* QZS 1:L1, 2:L2, 3:L5, 4:LEX */
	{"C", "IQX", "", "", "", "", ""},                  /* SBS 1:L1, 2:L5 */
	{"IQX", "IQX", "IQX", "", "", "", ""},             /* BDS 1:B1, 2:B3, 3:B2 */
	{"ABCX", "ABCX", "", "", "", "", ""}               /* IRN 1:L5, 2:S */
};

const double lam_carr[] = {/* carrier wave length (m) */
	CLIGHT / FREQ1, CLIGHT / FREQ2, CLIGHT / FREQ5, CLIGHT / FREQ6, 
	CLIGHT / FREQ7, CLIGHT / FREQ8, CLIGHT / FREQ9 
};

#ifdef INT_SEC_SEND
TIME_S sensor_time_s;
#endif

extern void set_glo_frq(unsigned char prn, int frq)
{
    int max_prn = sizeof(default_glo_frq_table) / sizeof(int);
    if (prn <= max_prn)
    {
        default_glo_frq_table[prn - 1] = frq;
    }
    return;
}

extern int get_glo_frq(unsigned char prn)
{
    int max_prn = sizeof(default_glo_frq_table) / sizeof(int);
    int frq = -99;
    if (prn <= max_prn)
    {
        frq = default_glo_frq_table[prn - 1];
    }
    return frq;
}

extern void set_week_number(int week)
{
    default_week_number = week;
    return;
}

extern int get_week_number()
{
    return default_week_number;
}

/* add time --------------------------------------------------------------------
* add time to gtime_t struct
* args   : gtime_t t        I   gtime_t struct
*          double sec       I   time to add (s)
* return : gtime_t struct (t+sec)
*-----------------------------------------------------------------------------*/
extern gtime_t timeadd(gtime_t t, double sec)
{
    double tt;

    t.sec += sec;
    tt = floor(t.sec);
    t.time += (int)tt;
    t.sec -= tt;
    return t;
}
/* time difference -------------------------------------------------------------
* difference between gtime_t structs
* args   : gtime_t t1,t2    I   gtime_t structs
* return : time difference (t1-t2) (s)
*-----------------------------------------------------------------------------*/
extern double timediff(gtime_t t1, gtime_t t2)
{
    return difftime(t1.time, t2.time) + t1.sec - t2.sec;
}
/* convert calendar day/time to time -------------------------------------------
* convert calendar day/time to gtime_t struct
* args   : double *ep       I   day/time {year,month,day,hour,min,sec}
* return : gtime_t struct
* notes  : proper in 1970-2037 or 1970-2099 (64bit time_t)
*-----------------------------------------------------------------------------*/
extern gtime_t epoch2time(const double *ep)
{
    const int doy[] = {1, 32, 60, 91, 121, 152, 182, 213, 244, 274, 305, 335};
    gtime_t time = {0};
    int days, sec, year = (int)ep[0], mon = (int)ep[1], day = (int)ep[2];

    if (year < 1970 || 2099 < year || mon < 1 || 12 < mon)
        return time;

    /* leap year if year%4==0 in 1901-2099 */
    days = (year - 1970) * 365 + (year - 1969) / 4 + doy[mon - 1] + day - 2 + (year % 4 == 0 && mon >= 3 ? 1 : 0);
    sec = (int)floor(ep[5]);
    time.time = (time_t)days * 86400 + (int)ep[3] * 3600 + (int)ep[4] * 60 + sec;
    time.sec = ep[5] - sec;
    return time;
}
/* time to calendar day/time ---------------------------------------------------
* convert gtime_t struct to calendar day/time
* args   : gtime_t t        I   gtime_t struct
*          double *ep       O   day/time {year,month,day,hour,min,sec}
* return : none
* notes  : proper in 1970-2037 or 1970-2099 (64bit time_t)
*-----------------------------------------------------------------------------*/
extern void time2epoch(gtime_t t, double *ep)
{
    const int mday[] = {/* # of days in a month */
		31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31,
		31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31
	};
    int days, sec, mon, day;

    /* leap year if year%4==0 in 1901-2099 */
    days = (int)(t.time / 86400);
    sec = (int)(t.time - (time_t)days * 86400);
    for (day = days % 1461, mon = 0; mon < 48; mon++)
    {
        if (day >= mday[mon])
            day -= mday[mon];
        else
            break;
    }
    ep[0] = 1970 + days / 1461 * 4 + mon / 12;
    ep[1] = mon % 12 + 1;
    ep[2] = day + 1;
    ep[3] = sec / 3600;
    ep[4] = sec % 3600 / 60;
    ep[5] = sec % 60 + t.sec;
}
/* beidou time (bdt) to time ---------------------------------------------------
* convert week and tow in beidou time (bdt) to gtime_t struct
* args   : int    week      I   week number in bdt
*          double sec       I   time of week in bdt (s)
* return : gtime_t struct
*-----------------------------------------------------------------------------*/
extern gtime_t bdt2time(int week, double sec)
{
    gtime_t t = epoch2time(bdt0);

    if (sec < -1E9 || 1E9 < sec)
        sec = 0.0;
    t.time += SECONDS_IN_WEEK * week + (int)sec;
    t.sec = sec - (int)sec;

    return t;
}
/* time to beidouo time (bdt) --------------------------------------------------
* convert gtime_t struct to week and tow in beidou time (bdt)
* args   : gtime_t t        I   gtime_t struct
*          int    *week     IO  week number in bdt (NULL: no output)
* return : time of week in bdt (s)
*-----------------------------------------------------------------------------*/
extern double time2bdt(gtime_t t, int *week)
{
    gtime_t t0 = epoch2time(bdt0);
    time_t sec = t.time - t0.time;
    int w = (int)(sec / SECONDS_IN_WEEK);

    if (week)
        *week = w;

    return (double)(sec - w * SECONDS_IN_WEEK) + t.sec;
}
/* time to gps time ------------------------------------------------------------
* convert gtime_t struct to week and tow in gps time
* args   : gtime_t t        I   gtime_t struct
*          int    *week     IO  week number in gps time (NULL: no output)
* return : time of week in gps time (s)
*-----------------------------------------------------------------------------*/
extern double time2gpst(gtime_t t, int *week)
{
    gtime_t t0 = epoch2time(gpst0);
    time_t sec = t.time - t0.time;
    int w = (int)(sec / SECONDS_IN_WEEK);

    if (week)
        *week = w;

    return (double)(sec - w * SECONDS_IN_WEEK) + t.sec;
}

/* utc to gpstime --------------------------------------------------------------
* convert utc to gpstime considering leap seconds
* args   : gtime_t t        I   time expressed in utc
* return : time expressed in gpstime
* notes  : ignore slight time offset under 100 ns
*-----------------------------------------------------------------------------*/
extern gtime_t utc2gpst(gtime_t t)
{
    return timeadd(t, 18.0);
}

/* gpstime to utc --------------------------------------------------------------
* convert gpstime to utc considering leap seconds
* args   : gtime_t t        I   time expressed in gpstime
* return : time expressed in utc
* notes  : ignore slight time offset under 100 ns
*-----------------------------------------------------------------------------*/
extern gtime_t gpst2utc(gtime_t t)
{
    return timeadd(t, -18.0);
}

/* gps time to time ------------------------------------------------------------
* convert week and tow in gps time to gtime_t struct
* args   : int    week      I   week number in gps time
*          double sec       I   time of week in gps time (s)
* return : gtime_t struct
*-----------------------------------------------------------------------------*/
extern gtime_t gpst2time(int week, double sec)
{
    gtime_t t = epoch2time(gpst0);

    if (sec < -1E9 || 1E9 < sec)
        sec = 0.0;
    t.time += SECONDS_IN_WEEK * week + (int)sec;
    t.sec = sec - (int)sec;

    return t;
}

/* gpstime to bdt --------------------------------------------------------------
* convert gpstime to bdt (beidou navigation satellite system time)
* args   : gtime_t t        I   time expressed in gpstime
* return : time expressed in bdt
* notes  : ref [8] 3.3, 2006/1/1 00:00 BDT = 2006/1/1 00:00 UTC
*          no leap seconds in BDT
*          ignore slight time offset under 100 ns
*-----------------------------------------------------------------------------*/
extern gtime_t gpst2bdt(gtime_t t)
{
    return timeadd(t, -14.0);
}
/* bdt to gpstime --------------------------------------------------------------
* convert bdt (beidou navigation satellite system time) to gpstime
* args   : gtime_t t        I   time expressed in bdt
* return : time expressed in gpstime
* notes  : see gpst2bdt()
*-----------------------------------------------------------------------------*/
extern gtime_t bdt2gpst(gtime_t t)
{
    return timeadd(t, 14.0);
}
/* time to string --------------------------------------------------------------
* convert gtime_t struct to string
* args   : gtime_t t        I   gtime_t struct
*          char   *s        O   string ("yyyy/mm/dd hh:mm:ss.ssss")
*          int    n         I   number of decimals
* return : none
*-----------------------------------------------------------------------------*/
extern void time2str(gtime_t t, char *s, int n)
{
    double ep[6];

    if (n < 0)
        n = 0;
    else if (n > 12)
        n = 12;
    if (1.0 - t.sec < 0.5 / pow(10.0, n))
    {
        t.time++;
        t.sec = 0.0;
    };
    time2epoch(t, ep);
    sprintf(s, "%04.0f/%02.0f/%02.0f %02.0f:%02.0f:%0*.*f", ep[0], ep[1], ep[2],
            ep[3], ep[4], n <= 0 ? 2 : n + 3, n <= 0 ? 0 : n, ep[5]);
}
/* get time string -------------------------------------------------------------
* get time string
* args   : gtime_t t        I   gtime_t struct
*          int    n         I   number of decimals
* return : time string
* notes  : not reentrant, do not use multiple in a function
*-----------------------------------------------------------------------------*/
extern char *time_str(gtime_t t, int n)
{
    static char buff[64];
    time2str(t, buff, n);
    return buff;
}

extern gtime_t timeget()
{
    return gpst2time(get_week_number(), 0.0);
}

/* adjust weekly rollover of gps time ----------------------------------------*/
extern void adjweek(gtime_t *time, double tow)
{
    double tow_p;
    int week;

    /* if no time, get cpu time */
    if (time->time == 0)
    {
        *time = utc2gpst(timeget());
    }
    else
    {
        tow_p = time2gpst(*time, &week);
        if (tow < tow_p - 302400.0)
            tow += 604800.0;
        else if (tow > tow_p + 302400.0)
            tow -= 604800.0;
        *time = gpst2time(week, tow);
    }
}
/* adjust gps week number ------------------------------------------------------
* adjust gps week number using cpu time
* args   : int   week       I   not-adjusted gps week number
* return : adjusted gps week number
*-----------------------------------------------------------------------------*/
static int adjgpsweek(gtime_t *time, int week)
{
    int w;

    if (time->time == 0)
        *time = utc2gpst(timeget());
    (void)time2gpst(*time, &w);
    //if (w<1560) w=1560; /* use 2009/12/1 if time is earlier than 2009/12/1 */

    return week + (w - week + 512) / 1024 * 1024;
}
/* adjust weekly rollover of bdt time ----------------------------------------*/
static int adjbdtweek(gtime_t *time, int week)
{
    int w;

    if (time->time == 0)
        *time = utc2gpst(timeget());
    (void)time2bdt(gpst2bdt(*time), &w);
    if (w < 1)
        w = 1; /* use 2006/1/1 if time is earlier than 2006/1/1 */

    return week + (w - week + 512) / 1024 * 1024;
}
/* adjust daily rollover of glonass time -------------------------------------*/
void adjday_glot(gtime_t *time, double tod)
{
    double tow, tod_p;
    int week;

    if (time->time == 0)
        *time = utc2gpst(timeget());
    *time = timeadd(gpst2utc(*time), 10800.0); /* glonass time */
    tow = time2gpst(*time, &week);
    tod_p = fmod(tow, 86400.0);
    tow -= tod_p;
    if (tod < tod_p - 43200.0)
        tod += 86400.0;
    else if (tod > tod_p + 43200.0)
        tod -= 86400.0;
    *time = gpst2time(week, tow + tod);
    *time = utc2gpst(timeadd(*time, -10800.0));
}

extern void trace(int level, const char *format, ...)
{
#ifdef _TRACE_
    va_list ap;

    char buffer[255];

    if (level > 0)
    {
        va_start(ap, format);
        vsprintf(buffer, format, ap);
        va_end(ap);
        printf("%s", buffer);
        //OutputDebugString(buffer);
    }
#endif
}

#ifdef QT_QML_DEBUG
FILE *logFile = NULL;
extern void OpenLogFile()
{
    if (logFile == NULL)
    {
        logFile = fopen("./test.log", "w+");
    }
}

extern void CloseLogFile()
{
    if (logFile)
    {
        fclose(logFile);
        logFile = NULL;
    }
}
#endif

/* crc-24q parity --------------------------------------------------------------
* compute crc-24q parity for sbas, rtcm3
* args   : unsigned char *buff I data
*          int    len    I      data length (bytes)
* return : crc-24Q parity
* notes  : see reference [2] A.4.3.3 Parity
*-----------------------------------------------------------------------------*/
unsigned int rtk_crc24q(const unsigned char *buff, int len)
{
    unsigned int crc = 0;
    int i;

    trace(4, "crc24q: len=%d\n", len);

    for (i = 0; i < len; i++)
        crc = ((crc << 8) & 0xFFFFFF) ^ tbl_CRC24Q[(crc >> 16) ^ buff[i]];

    return crc;
}

/* extract unsigned/signed bits ------------------------------------------------
* extract unsigned/signed bits from byte data
* args   : unsigned char *buff I byte data
*          int    pos    I      bit position from start of data (bits)
*          int    len    I      bit length (bits) (len<=32)
* return : extracted unsigned/signed bits
*-----------------------------------------------------------------------------*/
unsigned int rtcm_getbitu(const unsigned char *buff, int pos, int len)
{
    unsigned int bits = 0;
    int i;

    for (i = pos; i < pos + len; i++)
        bits = (bits << 1) + ((buff[i / 8] >> (7 - i % 8)) & 1u);

    return bits;
}
int rtcm_getbits(const unsigned char *buff, int pos, int len)
{
    unsigned int bits = rtcm_getbitu(buff, pos, len);

    if (len <= 0 || 32 <= len || !(bits & (1u << (len - 1))))
        return (int)bits;

    return (int)(bits | (~0u << len)); /* extend sign */
}
/* set unsigned/signed bits ----------------------------------------------------
* set unsigned/signed bits to byte data
* args   : unsigned char *buff IO byte data
*          int    pos    I      bit position from start of data (bits)
*          int    len    I      bit length (bits) (len<=32)
*         (unsigned) int I      unsigned/signed data
* return : none
*-----------------------------------------------------------------------------*/
void setbitu(unsigned char *buff, int pos, int len, unsigned int data)
{
    unsigned int mask = 1u << (len - 1);
    int i;

    if (len <= 0 || 32 < len)
        return;
    for (i = pos; i < pos + len; i++, mask >>= 1)
    {
        if (data & mask)
            buff[i / 8] |= 1u << (7 - i % 8);
        else
            buff[i / 8] &= ~(1u << (7 - i % 8));
    }
}
void setbits(unsigned char *buff, int pos, int len, int data)
{
    if (data < 0)
        data |= 1 << (len - 1);
    else
        data &= ~(1 << (len - 1)); /* set sign bit */
    setbitu(buff, pos, len, (unsigned int)data);
}

/* get sign-magnitude bits ---------------------------------------------------*/
static double getbitg(const unsigned char *buff, int pos, int len)
{
    double value = rtcm_getbitu(buff, pos + 1, len - 1);

    return rtcm_getbitu(buff, pos, 1) ? -value : value;
}
/* get signed 38bit field ----------------------------------------------------*/
static double rtcm_getbits_38(const unsigned char *buff, int pos)
{
    return (double)rtcm_getbits(buff, pos, 32) * 64.0 + rtcm_getbitu(buff, pos + 32, 6);
}
/* set signed 38bit field ----------------------------------------------------*/
void rtcm_setbits_38(unsigned char *buff, int pos, double data)
{
    int word_h = (int)floor(data / 64.0);
    unsigned int word_l = (unsigned int)(data - word_h * 64.0);
    
    setbits(buff, pos, 32, word_h);
    setbitu(buff, pos + 32, 6, word_l);
}

/* get observation data index ------------------------------------------------*/
static int obsindex(obs_t *obs, gtime_t time, int sat)
{
    unsigned int i;

    double tt = timediff(obs->time, time);

    if (fabs(tt) > 0.01)
    {
        /* new epoch, reset the n and obsflag */
        obs->n = 0;
        obs->obsflag = 0;
    }
    if (obs->n == 0)
    {
        /* first obs, set the time tag */
        obs->time = time;
    }
    for (i = 0; i < obs->n; i++)
    {
		if (obs->data[i].sat == sat)
			break; /* field already exists */
    }
	if (i == obs->n)
	{
		/* add new field */
		if (obs->n < MAXOBS)
		{
			memset(obs->data + i, 0, sizeof(obsd_t));
			obs->data[i].sat = (unsigned char)sat;
			obs->n++;
		}
		else
		{
			i = -1;
		}
	}
	else
	{
		/* duplicated satellites */
		memset(obs->data + i, 0, sizeof(obsd_t));
		obs->data[i].sat = (unsigned char)sat;
	}

    return i;
}
/* test station id consistency -----------------------------------------------*/
static int test_staid(obs_t *obs, int staid)
{
    /*int type;*/

    /* save station id */
    if (obs->staid == 0 || obs->obsflag)
    {
        obs->staid = staid;
    }
    else if (staid != obs->staid)
    {
        trace(2, "rtcm3 staid invalid id=%d %d\n", staid, obs->staid);

        /* reset station id if station id error */
        obs->staid = 0;
        return 0;
    }

    return 1;
}

/* satellite system+prn/slot number to satellite number ------------------------
* convert satellite system+prn/slot number to satellite number
* args   : int    sys       I   satellite system (SYS_GPS,SYS_GLO,...)
*          int    prn       I   satellite prn/slot number
* return : satellite number (0:error)
*-----------------------------------------------------------------------------*/
extern int satno(int sys, int prn)
{
    if (prn <= 0)
        return 0;

    switch (sys)
    {
    case _SYS_GPS_:
        if (prn < MINPRNGPS || MAXPRNGPS < prn)
            return 0;
        return prn - MINPRNGPS + 1;
    case _SYS_GLO_:
        if (prn < MINPRNGLO || MAXPRNGLO < prn)
            return 0;
        return NSATGPS + prn - MINPRNGLO + 1;
    case _SYS_GAL_:
        if (prn < MINPRNGAL || MAXPRNGAL < prn)
            return 0;
        return NSATGPS + NSATGLO + prn - MINPRNGAL + 1;
    case _SYS_QZS_:
        if (prn < MINPRNQZS || MAXPRNQZS < prn)
            return 0;
        return NSATGPS + NSATGLO + NSATGAL + prn - MINPRNQZS + 1;
    case _SYS_BDS_:
        if (prn < MINPRNCMP || MAXPRNCMP < prn)
            return 0;
        return NSATGPS + NSATGLO + NSATGAL + NSATQZS + prn - MINPRNCMP + 1;
    case _SYS_LEO_:
        if (prn < MINPRNLEO || MAXPRNLEO < prn)
            return 0;
        return NSATGPS + NSATGLO + NSATGAL + NSATQZS + NSATCMP + prn - MINPRNLEO + 1;
    case _SYS_SBS_:
        if (prn < MINPRNSBS || MAXPRNSBS < prn)
            return 0;
        return NSATGPS + NSATGLO + NSATGAL + NSATQZS + NSATCMP + NSATLEO + prn - MINPRNSBS + 1;
    }

    return 0;
}
/* satellite number to satellite system ----------------------------------------
* convert satellite number to satellite system
* args   : int    sat       I   satellite number (1-MAXSAT)
*          int    *prn      IO  satellite prn/slot number (NULL: no output)
* return : satellite system (SYS_GPS,SYS_GLO,...)
*-----------------------------------------------------------------------------*/
extern int satsys(int sat, int *prn)
{
    int sys = _SYS_NONE_;

    if (sat <= 0 || MAXSAT < sat)
        sat = 0;
    else if (sat <= NSATGPS)
    {
        sys = _SYS_GPS_;
        sat += MINPRNGPS - 1;
    }
    else if ((sat -= NSATGPS) <= NSATGLO)
    {
        sys = _SYS_GLO_;
        sat += MINPRNGLO - 1;
    }
    else if ((sat -= NSATGLO) <= NSATGAL)
    {
        sys = _SYS_GAL_;
        sat += MINPRNGAL - 1;
    }
    else if ((sat -= NSATGAL) <= NSATQZS)
    {
        sys = _SYS_QZS_;
        sat += MINPRNQZS - 1;
    }
    else if ((sat -= NSATQZS) <= NSATCMP)
    {
        sys = _SYS_BDS_;
        sat += MINPRNCMP - 1;
    }
    else if ((sat -= NSATCMP) <= NSATLEO)
    {
        sys = _SYS_LEO_;
        sat += MINPRNLEO - 1;
    }
    else if ((sat -= NSATLEO) <= NSATSBS)
    {
        sys = _SYS_SBS_;
        sat += MINPRNSBS - 1;
    }
    else
        sat = 0;
    if (prn)
        *prn = sat;

    return sys;
}

extern char satid(int sat, int *prn)
{
    char sys = ' ';

    if (sat <= 0 || MAXSAT < sat)
        sat = 0;
    else if (sat <= NSATGPS)
    {
        sys = 'G';
        sat += MINPRNGPS - 1;
    }
    else if ((sat -= NSATGPS) <= NSATGLO)
    {
        sys = 'R';
        sat += MINPRNGLO - 1;
    }
    else if ((sat -= NSATGLO) <= NSATGAL)
    {
        sys = 'E';
        sat += MINPRNGAL - 1;
    }
    else if ((sat -= NSATGAL) <= NSATQZS)
    {
        sys = 'J';
        sat += MINPRNQZS - 1;
    }
    else if ((sat -= NSATQZS) <= NSATCMP)
    {
        sys = 'C';
        sat += MINPRNCMP - 1;
    }
    else if ((sat -= NSATCMP) <= NSATLEO)
    {
        sys = 'L';
        sat += MINPRNLEO - 1;
    }
    else if ((sat -= NSATLEO) <= NSATSBS)
    {
        sys = 'S';
        sat += MINPRNSBS - 1;
    }
    else
        sat = 0;

    if (prn)
        *prn = sat;

    return sys;
}

/* only use GPS, GLO, GAL, BDS */
extern int satidx(int sat, int *prn)
{
    int sys = -1;

    if (sat <= 0 || MAXSAT < sat)
        sat = 0;
    else if (sat <= NSATGPS)
    {
        sys = 0;
        sat += MINPRNGPS - 1; /* GPS */
    }
    else if ((sat -= NSATGPS) <= NSATGLO)
    {
        sys = 3;
        sat += MINPRNGLO - 1; /* GLO */
    }
    else if ((sat -= NSATGLO) <= NSATGAL)
    {
        sys = 1;
        sat += MINPRNGAL - 1; /* GAL */
    }
    else if ((sat -= NSATGAL) <= NSATQZS)
    {
        sys = -1;
        sat += MINPRNQZS - 1; /* QZSS */
    }
    else if ((sat -= NSATQZS) <= NSATCMP)
    {
        sys = 2;
        sat += MINPRNCMP - 1; /* BDS */
    }
    else if ((sat -= NSATCMP) <= NSATLEO)
    {
        sys = -1;
        sat += MINPRNLEO - 1; /* LEO */
    }
    else if ((sat -= NSATLEO) <= NSATSBS)
    {
        sys = -1;
        sat += MINPRNSBS - 1; /* SBS */
    }
    else
        sat = 0;
    if (prn)
        *prn = sat;
    return sys;
}

extern char sys2char(int sys)
{
    int s_char = ' ';

    if (sys == _SYS_GPS_)
        s_char = 'G';
    else if (sys == _SYS_GLO_)
        s_char = 'R';
    else if (sys == _SYS_GAL_)
        s_char = 'E';
    else if (sys == _SYS_QZS_)
        s_char = 'J';
    else if (sys == _SYS_BDS_)
        s_char = 'C';
    else if (sys == _SYS_LEO_)
        s_char = 'L';
    else if (sys == _SYS_SBS_)
        s_char = 'S';

    return s_char;
}

/* satellite carrier wave length -----------------------------------------------
* get satellite carrier wave lengths
* args   : int    sat       I   satellite number
*          int    frq       I   frequency index (0:L1,1:L2,2:L5/3,...)
*          nav_t  *nav      I   navigation messages
* return : carrier wave length (m) (0.0: error)
*-----------------------------------------------------------------------------*/
extern double satwavelen(int sat, int frq)
{
    const double freq_glo[] = {FREQ1_GLO, FREQ2_GLO};
    const double dfrq_glo[] = {DFRQ1_GLO, DFRQ2_GLO};
    int prn = 0, sys = satsys(sat, &prn);
    int frqnum = get_glo_frq(prn);

    if (sys == _SYS_GLO_)
    {
        if (frqnum == -99)
            return 0.0;
        if (0 <= frq && frq <= 1)
        {
            return CLIGHT / (freq_glo[frq] + dfrq_glo[frq] * frqnum);
        }
        else if (frq == 2)
        { /* L3 */
            return CLIGHT / FREQ3_GLO;
        }
    }
    else if (sys == _SYS_BDS_)
    {
        if (frq == 0)
            return CLIGHT / FREQ1_CMP; /* B1 */
        else if (frq == 1)
            return CLIGHT / FREQ3_CMP; /* B3 */
        else if (frq == 2)
            return CLIGHT / FREQ2_CMP; /* B2 */
    }
    else if (sys == _SYS_GAL_)
    {
        if (frq == 0)
            return CLIGHT / FREQ1; /* E1 */
        else if (frq == 1)
            return CLIGHT / FREQ7; /* E5b */
        else if (frq == 2)
            return CLIGHT / FREQ5; /* E5a */
        else if (frq == 3)
            return CLIGHT / FREQ8; /* E5a+b */
        else if (frq == 4)
            return CLIGHT / FREQ6; /* E6 */
    }
    else if (sys == _SYS_QZS_)
    {
        if (frq == 0)
            return CLIGHT / FREQ1; /* L1 */
        else if (frq == 1)
            return CLIGHT / FREQ2; /* L2 */
        else if (frq == 2)
            return CLIGHT / FREQ5; /* L5 */
        else if (frq == 3)
            return CLIGHT / FREQ6; /* LEX */
    }
    else if (sys == _SYS_GPS_)
    {
        if (frq == 0)
            return CLIGHT / FREQ1; /* L1 */
        else if (frq == 1)
            return CLIGHT / FREQ2; /* L2 */
        else if (frq == 2)
            return CLIGHT / FREQ5; /* L5 */
    }

    return 0.0;
}

/* obs type string to obs code -------------------------------------------------
* convert obs code type string to obs code
* args   : char   *str   I      obs code string ("1C","1P","1Y",...)
*          int    *freq  IO     frequency (1:L1,2:L2,3:L5,4:L6,5:L7,6:L8,0:err)
*                               (NULL: no output)
* return : obs code (CODE_???)
* notes  : obs codes are based on reference [6] and qzss extension
*-----------------------------------------------------------------------------*/
extern unsigned char obs2code(int sys, const char *obs, int *freq)
{
    int i;

    if (freq)
        *freq = 0;

    for (i = 1; *obscodes[i]; i++)
    {
        if (strcmp(obscodes[i], obs))
            continue;
        if (freq)
        {
            if (sys == _SYS_GPS_)
                *freq = obsfreqs_gps[i];
            else if (sys == _SYS_GLO_)
                *freq = obsfreqs_glo[i];
            else if (sys == _SYS_GAL_)
                *freq = obsfreqs_gal[i];
            else if (sys == _SYS_QZS_)
                *freq = obsfreqs_qzs[i];
            else if (sys == _SYS_SBS_)
                *freq = obsfreqs_sbs[i];
            else if (sys == _SYS_BDS_)
                *freq = obsfreqs_cmp[i];
            else if (sys == _SYS_IRN_)
                *freq = obsfreqs_irn[i];
        }
        return (unsigned char)i;
    }

    return CODE_NONE;
}
/* obs code to obs code string -------------------------------------------------
* convert obs code to obs code string
* args   : unsigned char code I obs code (CODE_???)
*          int    *freq  IO     frequency (NULL: no output)
*                               (1:L1/E1, 2:L2/B1, 3:L5/E5a/L3, 4:L6/LEX/B3,
                                 5:E5b/B2, 6:E5(a+b), 7:S)
* return : obs code string ("1C","1P","1P",...)
* notes  : obs codes are based on reference [6] and qzss extension
*-----------------------------------------------------------------------------*/
extern char *code2obs(int sys, unsigned char code, int *freq)
{
    if (freq)
        *freq = 0;

    if (code <= CODE_NONE || MAXCODE < code)
        return "";

    if (freq)
    {
        if (sys == _SYS_GPS_)
            *freq = obsfreqs_gps[code];
        else if (sys == _SYS_GLO_)
            *freq = obsfreqs_glo[code];
        else if (sys == _SYS_GAL_)
            *freq = obsfreqs_gal[code];
        else if (sys == _SYS_QZS_)
            *freq = obsfreqs_qzs[code];
        else if (sys == _SYS_SBS_)
            *freq = obsfreqs_sbs[code];
        else if (sys == _SYS_BDS_)
            *freq = obsfreqs_cmp[code];
        else if (sys == _SYS_IRN_)
            *freq = obsfreqs_irn[code];
    }

    return obscodes[code];
}
/* set code priority -----------------------------------------------------------
* set code priority for multiple codes in a frequency
* args   : int    sys     I     system (or of SYS_???)
*          int    freq    I     frequency (1:L1,2:L2,3:L5,4:L6,5:L7,6:L8,7:L9)
*          char   *pri    I     priority of codes (series of code characters)
*                               (higher priority precedes lower)
* return : none
*-----------------------------------------------------------------------------*/
extern void setcodepri(int sys, int freq, const char *pri)
{
    trace(3, "setcodepri:sys=%c freq=%d pri=%s\n", sys2char(sys), freq, pri);

    if (freq <= 0 || MAXFREQ < freq)
        return;
    if (sys & _SYS_GPS_)
        strcpy(codepris[0][freq - 1], pri);
    if (sys & _SYS_GLO_)
        strcpy(codepris[1][freq - 1], pri);
    if (sys & _SYS_GAL_)
        strcpy(codepris[2][freq - 1], pri);
    if (sys & _SYS_QZS_)
        strcpy(codepris[3][freq - 1], pri);
    if (sys & _SYS_SBS_)
        strcpy(codepris[4][freq - 1], pri);
    if (sys & _SYS_BDS_)
        strcpy(codepris[5][freq - 1], pri);
    if (sys & _SYS_IRN_)
        strcpy(codepris[6][freq - 1], pri);
}
/* get code priority -----------------------------------------------------------
* get code priority for multiple codes in a frequency
* args   : int    sys     I     system (SYS_???)
*          unsigned char code I obs code (CODE_???)
*          char   *opt    I     code options (NULL:no option)
* return : priority (15:highest-1:lowest,0:error)
*-----------------------------------------------------------------------------*/
extern int getcodepri(int sys, unsigned char code, const char *opt)
{
    const char *p, *optstr;
    char *obs, str[8] = "";
    int i, j;

    switch (sys)
    {
    case _SYS_GPS_:
        i = 0;
        optstr = "-GL%2s";
        break;
    case _SYS_GLO_:
        i = 1;
        optstr = "-RL%2s";
        break;
    case _SYS_GAL_:
        i = 2;
        optstr = "-EL%2s";
        break;
    case _SYS_QZS_:
        i = 3;
        optstr = "-JL%2s";
        break;
    case _SYS_SBS_:
        i = 4;
        optstr = "-SL%2s";
        break;
    case _SYS_BDS_:
        i = 5;
        optstr = "-CL%2s";
        break;
    case _SYS_IRN_:
        i = 6;
        optstr = "-IL%2s";
        break;
    default:
        return 0;
    }

    obs = code2obs(sys, code, &j);

    /* parse code options */
    for (p = opt; p && (p = strchr(p, '-')); p++)
    {
        if (sscanf(p, optstr, str) < 1 || str[0] != obs[0])
            continue;
        return str[1] == obs[1] ? 15 : 0;
    }

    /* search code priority */
    return (p = strchr(codepris[i][j - 1], obs[1])) ? 14 - (int)(p - codepris[i][j - 1]) : 0;
}

/* get signal index ----------------------------------------------------------*/
static void sigindex(int sys, const unsigned char *code, const int *freq, int n,
                     const char *opt, int *ind)
{
    int i, nex, pri, pri_h[8] = {0}, index[8] = {0}, ex[32] = {0};

    /* test code priority */
    for (i = 0; i < n; i++)
    {
        if (!code[i])
            continue;

        if (freq[i] > NFREQ)
        { /* save as extended signal if freq > NFREQ */
            ex[i] = 1;
            continue;
        }
        /* code priority */
        pri = getcodepri(sys, code[i], opt);

        /* select highest priority signal */
        if (pri > pri_h[freq[i] - 1])
        {
            if (index[freq[i] - 1])
                ex[index[freq[i] - 1] - 1] = 1;
            pri_h[freq[i] - 1] = pri;
            index[freq[i] - 1] = i + 1;
        }
        else
            ex[i] = 1;
    }
    /* signal index in obs data */
    for (i = nex = 0; i < n; i++)
    {
        if (ex[i] == 0)
            ind[i] = freq[i] - 1;
        else if (nex < NEXOBS)
            ind[i] = NFREQ + nex++;
        else
        { /* no space in obs data */
            trace(2, "rtcm msm: no space in obs data sys=%c code=%d\n", sys2char(sys), code[i]);
            ind[i] = -1;
        }
#if 0
        trace(2,"sig pos: sys=%c code=%d ex=%d ind=%d\n", sys2char(sys),code[i],ex[i],ind[i]);
#endif
    }
}

/* adjust carrier-phase rollover ---------------------------------------------*/
static double adjcp(rtcm_t *rtcm, int sat, int freq, double cp)
{
    //if (rtcm->cp[sat-1][freq]==0.0) ;
    //else if (cp<rtcm->cp[sat-1][freq]-750.0) cp+=1500.0;
    //else if (cp>rtcm->cp[sat-1][freq]+750.0) cp-=1500.0;
    //rtcm->cp[sat-1][freq]=cp;
    return cp;
}
/* loss-of-lock indicator ----------------------------------------------------*/
static int lossoflock(rtcm_t *rtcm, int sat, int freq, int lock)
{
	int lli = (!lock && !rtcm->lock[sat - 1][freq]) || lock < rtcm->lock[sat - 1][freq];

	rtcm->lock[sat - 1][freq] = (unsigned short)lock;

	return lli;
}
/* s/n ratio -----------------------------------------------------------------*/
static unsigned char snratio(double snr)
{
    return (unsigned char)(snr <= 0.0 || 255.5 <= snr ? 0.0 : snr * 4.0 + 0.5);
}
/* add obsd to obs */
static int add_obs(obsd_t* obsd, obs_t* obs)
{
	unsigned int i;

	double tt = timediff(obs->time, obsd->time);

	if (fabs(tt) > 0.01)
	{
		/* new epoch, reset the n and obsflag */
		obs->n = 0;
		obs->obsflag = 0;
	}
	if (obs->n == 0)
	{
		/* first obs, set the time tag */
		obs->time = obsd->time;
	}
	for (i = 0; i < obs->n; i++)
	{
		if (obs->data[i].sat == obsd->sat)
			break; /* field already exists */
	}
	if (i == obs->n)
	{
		/* add new field */
		if (obs->n < MAXOBS)
		{
			obs->data[i] = *obsd;
			obs->n++;
		}
		else
		{
			i = -1;
		}
	}
	else
	{
		/* duplicated satellites */
		obs->data[i] = *obsd;
	}

	return i;
}
/* add eph to nav */
static int add_eph(eph_t* eph, nav_t* nav)
{
	int i = 0, ret = 0;
	int sat = eph->sat;
	int bestL = -1;
	double bestT = 0.0;

	if (sat <= 0)
	{
		return 0;
	}

	for (i = 0; i < nav->n; ++i)
	{
		if (nav->eph[i].sat == sat)
		{
			break;
		}
	}
	if (i < nav->n)
	{
		/* replace old */
		nav->eph[i] = *eph;
		nav->ephsat = sat;
	}
	else if (i == nav->n)
	{
		if (i < MAXEPH)
		{
			nav->eph[nav->n] = *eph;
			nav->ephsat = sat;
			++nav->n;
			ret = 1;
		}
		else
		{
			/* remove the oldest one */
			for (i = 0; i < nav->n; ++i)
			{
				double diffT = fabs(timediff(nav->eph[i].toe, eph->toe));
				if (bestL < 0 || bestT>diffT)
				{
					bestL = i;
					bestT = diffT;
				}
			}
			if (bestL >= 0)
			{
				nav->eph[bestL] = *eph;
				nav->ephsat = sat;
				ret = 1;
			}
		}
	}

	return ret;
}
/* add eph to nav */
static int add_geph(geph_t* eph, nav_t* nav)
{
	int i = 0, ret = 0;
	int sat = eph->sat;
	int bestL = -1;
	double bestT = 0.0;

	if (sat <= 0)
	{
		return 0;
	}

	for (i = 0; i < nav->ng; ++i)
	{
		if (nav->geph[i].sat == sat)
		{
			break;
		}
	}
	if (i < nav->ng)
	{
		/* replace old */
		nav->geph[i] = *eph;
		nav->ephsat = sat;
	}
	else if (i == nav->ng)
	{
		if (i < MAXEPH_R)
		{
			nav->geph[nav->ng] = *eph;
			nav->ephsat = sat;
			++nav->ng;
			ret = 1;
		}
		else
		{
			/* remove the oldest one */
			for (i = 0; i < nav->ng; ++i)
			{
				double diffT = fabs(timediff(nav->geph[i].toe, eph->toe));
				if (bestL < 0 || bestT>diffT)
				{
					bestL = i;
					bestT = diffT;
				}
			}
			if (bestL >= 0)
			{
				nav->geph[bestL] = *eph;
				nav->ephsat = sat;
				ret = 1;
			}
		}
	}

	return ret;
}

static void decode_type999_subtype4(rtcm_t *rtcm, obs_t *obs)
{
    int i = 44;
    rtcm->st_pvt.ref_station_id = rtcm_getbitu(rtcm->buff, i, 12);
    i += 12;
    rtcm->st_pvt.reserved_itrf = rtcm_getbitu(rtcm->buff, i, 6);
    i += 6;
    rtcm->st_pvt.gnss_quality_indicator = rtcm_getbitu(rtcm->buff, i, 4);
    i += 4;
    rtcm->st_pvt.num_sat_in_use = rtcm_getbitu(rtcm->buff, i, 8);
    i += 8;
    rtcm->st_pvt.num_sat_in_view = rtcm_getbitu(rtcm->buff, i, 8);
    i += 8;
    rtcm->st_pvt.hdop = (float)rtcm_getbitu(rtcm->buff, i, 8) * 0.1;
    i += 8;
    rtcm->st_pvt.vdop = (float)rtcm_getbitu(rtcm->buff, i, 8) * 0.1;
    i += 8;
    rtcm->st_pvt.pdop = (float)rtcm_getbitu(rtcm->buff, i, 8) * 0.1;
    i += 8;
    rtcm->st_pvt.geoidal_separation = (float)rtcm_getbitu(rtcm->buff, i, 15) * 0.01;
    i += 15;
    rtcm->st_pvt.diff_age = rtcm_getbitu(rtcm->buff, i, 24);
    i += 24;
    rtcm->st_pvt.diff_ref_station_id = rtcm_getbitu(rtcm->buff, i, 12);
    i += 12;
    rtcm->st_pvt.gnss_id = rtcm_getbitu(rtcm->buff, i, 4);
    i += 4;
    rtcm->st_pvt.gnss_epoch_time = rtcm_getbitu(rtcm->buff, i, 30);
    i += 30;
    rtcm->st_pvt.ext_gps_week_number = rtcm_getbitu(rtcm->buff, i, 16);
    i += 16;
    rtcm->st_pvt.leap_seconds = rtcm_getbitu(rtcm->buff, i, 8);
    i += 8;
    rtcm->st_pvt.pos_xyz[0] = (double)rtcm_getbits_38(rtcm->buff, i) *0.0001;
    i += 38;
    rtcm->st_pvt.pos_xyz[1] = (double)rtcm_getbits_38(rtcm->buff, i) *0.0001;
    i += 38;
    rtcm->st_pvt.pos_xyz[2] = (double)rtcm_getbits_38(rtcm->buff, i) *0.0001;
    i += 38;
    rtcm->st_pvt.vel_xyz[0] = (float)rtcm_getbits(rtcm->buff, i, 32) * 0.000001;
    i += 32;
    rtcm->st_pvt.vel_xyz[1] = (float)rtcm_getbits(rtcm->buff, i, 32) * 0.000001;
    i += 32;
    rtcm->st_pvt.vel_xyz[2] = (float)rtcm_getbits(rtcm->buff, i, 32) * 0.000001;
    i += 32;

	uint8_t j;
	for (j = 0; j < 3; j++)
	{
		obs->pos[j] = rtcm->st_pvt.pos_xyz[j];
	}    

    rtcm->st_pvt.flag_gnss_update = 1;
}

void decode_type999_subtype21(rtcm_t *rtcm, obs_t *obs)
{
    int i = 44;
    rtcm->st_epvt.reference_station_id = rtcm_getbitu(rtcm->buff, i, 12);
    i += 12;
    rtcm->st_epvt.reserved_itrf = rtcm_getbitu(rtcm->buff, i, 6);
    i += 6;
    rtcm->st_epvt.GPS_quality_indicator = rtcm_getbitu(rtcm->buff, i, 4);
    i += 4;
    rtcm->st_epvt.number_satellites_use = rtcm_getbitu(rtcm->buff, i, 8);
    i += 8;
    rtcm->st_epvt.number_satellites_view = rtcm_getbitu(rtcm->buff, i, 8);
    i += 8;
    rtcm->st_epvt.hdop = (float)rtcm_getbitu(rtcm->buff, i, 8) * 0.1;
    i += 8;
    rtcm->st_epvt.vdop = (float)rtcm_getbitu(rtcm->buff, i, 8) * 0.1;
    i += 8;
    rtcm->st_epvt.pdop = (float)rtcm_getbitu(rtcm->buff, i, 8) * 0.1;
    i += 8;
    rtcm->st_epvt.geoidal_separation = (float)rtcm_getbitu(rtcm->buff, i, 15) * 0.01;
    i += 15;
    rtcm->st_epvt.age_differentials = rtcm_getbitu(rtcm->buff, i, 24);
    i += 24;
    rtcm->st_epvt.differential_reference_station_id = rtcm_getbitu(rtcm->buff, i, 12);
    i += 12;
    rtcm->st_epvt.gnss_id = rtcm_getbitu(rtcm->buff, i, 4);
    i += 4;
    rtcm->st_epvt.gnss_epoch_time = rtcm_getbitu(rtcm->buff, i, 30);
    i += 30;
    rtcm->st_epvt.extended_week_number = rtcm_getbitu(rtcm->buff, i, 16);
    i += 16;
    rtcm->st_epvt.leap_seconds = rtcm_getbitu(rtcm->buff, i, 8);
    i += 8;
    rtcm->st_epvt.latitude = (double)rtcm_getbits(rtcm->buff, i, 32) * 0.001 / 3600;
    i += 32;
    rtcm->st_epvt.longitude = (double)rtcm_getbits(rtcm->buff, i, 32) * 0.001 / 3600;
    i += 32;
    rtcm->st_epvt. height= (double)rtcm_getbits(rtcm->buff, i, 16) * 0.1;
    i += 16;
    rtcm->st_epvt.velocity_horizontal = (double)rtcm_getbitu(rtcm->buff, i, 16) * 0.1;
    i += 16;
    rtcm->st_epvt.velocity_vertical = (double)rtcm_getbits(rtcm->buff, i, 16) * 0.1;
    i += 16;
    rtcm->st_epvt.course_angle = rtcm_getbitu(rtcm->buff, i, 16);
    i += 16;
    rtcm->st_epvt.protection_level_horizontal = (double)rtcm_getbits(rtcm->buff, i, 16) * 0.1;
    i += 16;   
    rtcm->st_epvt.protection_level_vertical = (double)rtcm_getbits(rtcm->buff, i, 16) * 0.1;
    i += 16; 
    rtcm->st_epvt.protection_level_angle = (double)rtcm_getbits(rtcm->buff, i, 16) * 0.1;
    i += 16;
    rtcm->st_epvt.receiver_clock_bias = rtcm_getbits(rtcm->buff, i, 32);
    i += 32;    
    rtcm->st_epvt.receiver_clock_drift = rtcm_getbits(rtcm->buff, i, 32);
    i += 32;  


	double r[3];
	r[0] =  rtcm->st_epvt.latitude * D2R;
	r[1] =  rtcm->st_epvt.longitude * D2R;
	r[2] =  rtcm->st_epvt.height;
	pos2ecef(r,obs->pos);

    rtcm->st_epvt.flag_gnss_update = 1;

}

static int decode_type999(rtcm_t *rtcm, obs_t *obs)
{

    int i = 24;
    rtcm->st_pvt.message_number = rtcm_getbitu(rtcm->buff, i, 12);
    i += 12;
    rtcm->st_pvt.sub_type_id = rtcm_getbitu(rtcm->buff, i, 8);
    i += 8;
    if (rtcm->st_pvt.sub_type_id == 4)
    {
        decode_type999_subtype4(rtcm, obs);
    }
    else  if (rtcm->st_pvt.sub_type_id == 21)
    {
        decode_type999_subtype21(rtcm, obs);
    }  
    return 0;
}

/* decode type 1001-1004 message header --------------------------------------*/
static int decode_head1001(rtcm_t *rtcm, obs_t *obs, int *sync)
{
    double tow;
    char tstr[64];
    unsigned int i = 24;
    int staid, nsat, type;

    type = rtcm_getbitu(rtcm->buff, i, 12);
    i += 12;

    if (i + 52 <= rtcm->len * 8)
    {
        staid = rtcm_getbitu(rtcm->buff, i, 12);
        i += 12;
        tow = rtcm_getbitu(rtcm->buff, i, 30) * 0.001;
        i += 30;
        *sync = rtcm_getbitu(rtcm->buff, i, 1);
        i += 1;
        nsat = rtcm_getbitu(rtcm->buff, i, 5);
    }
    else
    {
        trace(2, "rtcm3 %d length error: len=%d\n", type, rtcm->len);
        return -1;
    }
    /* test station id */
    if (!test_staid(obs, staid))
        return -1;

    adjweek(&rtcm->time, tow);

    time2str(rtcm->time, tstr, 2);
    trace(4, "decode_head1001: time=%s nsat=%d sync=%d\n", tstr, nsat, *sync);

    return nsat;
}
/* decode type 1001: L1-only gps rtk observation -----------------------------*/
static int decode_type1001(rtcm_t *rtcm, obs_t *obs)
{
    int sync;
    if (decode_head1001(rtcm, obs, &sync) < 0)
        return -1;
    obs->obsflag = !sync;
    return sync ? 0 : 1;
}
/* decode type 1002: extended L1-only gps rtk observables --------------------*/
static int decode_type1002(rtcm_t *rtcm, obs_t *obs)
{
    double pr1, cnr1, /*tt,*/ cp1;
    unsigned int i = 24 + 64, j = 0;
    int index, nsat, sync, prn, code, sat, ppr1, lock1, amb, sys;

    if ((nsat = decode_head1001(rtcm, obs, &sync)) < 0)
        return -1;

    for (j = 0; j < nsat && obs->n < MAXOBS && i + 74 <= rtcm->len * 8; j++)
    {
        prn = rtcm_getbitu(rtcm->buff, i, 6);
        i += 6;
        code = rtcm_getbitu(rtcm->buff, i, 1);
        i += 1;
        pr1 = rtcm_getbitu(rtcm->buff, i, 24);
        i += 24;
        ppr1 = rtcm_getbits(rtcm->buff, i, 20);
        i += 20;
        lock1 = rtcm_getbitu(rtcm->buff, i, 7);
        i += 7;
        amb = rtcm_getbitu(rtcm->buff, i, 8);
        i += 8;
        cnr1 = rtcm_getbitu(rtcm->buff, i, 8);
        i += 8;
        if (prn < 40)
        {
            sys = _SYS_GPS_;
        }
        else
        {
            sys = _SYS_SBS_;
            prn += 80;
        }
        if (!(sat = satno(sys, prn)))
        {
            trace(2, "rtcm3 1002 satellite number error: prn=%d\n", prn);
            continue;
        }
        if ((index = obsindex(obs, rtcm->time, sat)) < 0)
            continue;
        pr1 = pr1 * 0.02 + amb * PRUNIT_GPS;
        if (ppr1 != (int)0xFFF80000)
        {
            obs->data[index].P[0] = pr1;
            cp1 = adjcp(rtcm, sat, 0, ppr1 * 0.0005 / lam_carr[0]);
            obs->data[index].L[0] = pr1 / lam_carr[0] + cp1;
        }
        obs->data[index].LLI[0] = (unsigned char)lossoflock(rtcm, sat, 0, lock1);
        obs->data[index].SNR[0] = snratio(cnr1 * 0.25);
        obs->data[index].code[0] = code ? CODE_L1P : CODE_L1C;
    }
    return sync ? 0 : 1;
}
/* decode type 1003: L1&L2 gps rtk observables -------------------------------*/
static int decode_type1003(rtcm_t *rtcm, obs_t *obs)
{
    int sync;
    if (decode_head1001(rtcm, obs, &sync) < 0)
        return -1;
    obs->obsflag = !sync;
    return sync ? 0 : 1;
}
/* decode type 1004: extended L1&L2 gps rtk observables ----------------------*/
static int decode_type1004(rtcm_t *rtcm, obs_t *obs)
{
    const int L2codes[] = {CODE_L2X, CODE_L2P, CODE_L2D, CODE_L2W};
    double pr1, cnr1, cnr2, /*tt,*/ cp1, cp2;
    int i = 24 + 64, j, index, nsat, sync, prn, sat, code1, code2, pr21, ppr1, ppr2;
    int lock1, lock2, amb, sys;

    if ((nsat = decode_head1001(rtcm, obs, &sync)) < 0)
        return -1;

    for (j = 0; j < nsat && obs->n < MAXOBS && i + 125 <= rtcm->len * 8; j++)
    {
        prn = rtcm_getbitu(rtcm->buff, i, 6);
        i += 6;
        code1 = rtcm_getbitu(rtcm->buff, i, 1);
        i += 1;
        pr1 = rtcm_getbitu(rtcm->buff, i, 24);
        i += 24;
        ppr1 = rtcm_getbits(rtcm->buff, i, 20);
        i += 20;
        lock1 = rtcm_getbitu(rtcm->buff, i, 7);
        i += 7;
        amb = rtcm_getbitu(rtcm->buff, i, 8);
        i += 8;
        cnr1 = rtcm_getbitu(rtcm->buff, i, 8);
        i += 8;
        code2 = rtcm_getbitu(rtcm->buff, i, 2);
        i += 2;
        pr21 = rtcm_getbits(rtcm->buff, i, 14);
        i += 14;
        ppr2 = rtcm_getbits(rtcm->buff, i, 20);
        i += 20;
        lock2 = rtcm_getbitu(rtcm->buff, i, 7);
        i += 7;
        cnr2 = rtcm_getbitu(rtcm->buff, i, 8);
        i += 8;
        if (prn < 40)
        {
            sys = _SYS_GPS_;
        }
        else
        {
            sys = _SYS_SBS_;
            prn += 80;
        }
        if (!(sat = satno(sys, prn)))
        {
            trace(2, "rtcm3 1004 satellite number error: sys=%c prn=%d\n", sys2char(sys), prn);
            continue;
        }
        if ((index = obsindex(obs, rtcm->time, sat)) < 0)
            continue;
        pr1 = pr1 * 0.02 + amb * PRUNIT_GPS;
        if (ppr1 != (int)0xFFF80000)
        {
            obs->data[index].P[0] = pr1;
            cp1 = adjcp(rtcm, sat, 0, ppr1 * 0.0005 / lam_carr[0]);
            obs->data[index].L[0] = pr1 / lam_carr[0] + cp1;
        }
        obs->data[index].LLI[0] = (unsigned char)lossoflock(rtcm, sat, 0, lock1);
        obs->data[index].SNR[0] = snratio(cnr1 * 0.25);
        obs->data[index].code[0] = code1 ? CODE_L1P : CODE_L1C;

        if (pr21 != (int)0xFFFFE000)
        {
            obs->data[index].P[1] = pr1 + pr21 * 0.02;
        }
        if (ppr2 != (int)0xFFF80000)
        {
            cp2 = adjcp(rtcm, sat, 1, ppr2 * 0.0005 / lam_carr[1]);
            obs->data[index].L[1] = pr1 / lam_carr[1] + cp2;
        }
        obs->data[index].LLI[1] = (unsigned char)lossoflock(rtcm, sat, 1, lock2);
        obs->data[index].SNR[1] = snratio(cnr2 * 0.25);
        obs->data[index].code[1] = (unsigned char)L2codes[code2];
    }
    obs->obsflag = !sync;

    return sync ? 0 : 1;
}

/* decode type 1005: stationary rtk reference station arp --------------------*/
static int decode_type1005(rtcm_t *rtcm, obs_t *obs)
{
    double rr[3];
    int i = 24 + 12, j, staid /*,itrf*/;

    if (i + 140 == rtcm->len * 8)
    {
        staid = rtcm_getbitu(rtcm->buff, i, 12);
        i += 12;
        /*itrf =rtcm_getbitu(rtcm->buff,i, 6);*/ i += 6 + 4;
        rr[0] = rtcm_getbits_38(rtcm->buff, i);
        i += 38 + 2;
        rr[1] = rtcm_getbits_38(rtcm->buff, i);
        i += 38 + 2;
        rr[2] = rtcm_getbits_38(rtcm->buff, i);
    }
    else
    {
        trace(2, "rtcm3 1005 length error: len=%d\n", rtcm->len);
        return -1;
    }
    /* test station id */
    if (!test_staid(obs, staid))
        return -1;

    for (j = 0; j < 3; j++)
    {
        obs->pos[j] = rr[j] * 0.0001;
    }

    return 5;
}
/* decode type 1006: stationary rtk reference station arp with height --------*/
static int decode_type1006(rtcm_t *rtcm, obs_t *obs)
{
    double rr[3] /*,anth*/;
    int i = 24 + 12, j, staid, itrf;

    if (i + 156 <= rtcm->len * 8)
    {
        staid = rtcm_getbitu(rtcm->buff, i, 12);
        i += 12;
        itrf = rtcm_getbitu(rtcm->buff, i, 6);
        i += 6 + 4;
        rr[0] = rtcm_getbits_38(rtcm->buff, i);
        i += 38 + 2;
        rr[1] = rtcm_getbits_38(rtcm->buff, i);
        i += 38 + 2;
        rr[2] = rtcm_getbits_38(rtcm->buff, i);
        i += 38;
        /*anth =rtcm_getbitu(rtcm->buff,i,16);*/
    }
    else
    {
        trace(2, "rtcm3 1006 length error: len=%d\n", rtcm->len);
        return -1;
    }
    /* test station id */
    if (!test_staid(obs, staid))
        return -1;

    for (j = 0; j < 3; j++)
    {
        obs->pos[j] = rr[j] * 0.0001;
    }

    return 5;
}
/* decode type 1007: antenna descriptor --------------------------------------*/
static int decode_type1007(rtcm_t *rtcm, obs_t *obs)
{
    char des[32] = "";
    int i = 24 + 12, j, staid, n, setup;

    n = rtcm_getbitu(rtcm->buff, i + 12, 8);

    if (i + 28 + 8 * n <= rtcm->len * 8)
    {
        staid = rtcm_getbitu(rtcm->buff, i, 12);
        i += 12 + 8;
        for (j = 0; j < n && j < 31; j++)
        {
            des[j] = (char)rtcm_getbitu(rtcm->buff, i, 8);
            i += 8;
        }
        setup = rtcm_getbitu(rtcm->buff, i, 8);
    }
    else
    {
        trace(2, "rtcm3 1007 length error: len=%d\n", rtcm->len);
        return -1;
    }
    /* test station id */
    if (!test_staid(obs, staid))
        return -1;

    return 5;
}
/* decode type 1008: antenna descriptor & serial number ----------------------*/
static int decode_type1008(rtcm_t *rtcm, obs_t *obs)
{
    /*char des[32]="",sno[32]="";*/
    int i = 24 + 12, j, staid, n, m;

    n = rtcm_getbitu(rtcm->buff, i + 12, 8);
    m = rtcm_getbitu(rtcm->buff, i + 28 + 8 * n, 8);

    if (i + 36 + 8 * (n + m) <= rtcm->len * 8)
    {
        staid = rtcm_getbitu(rtcm->buff, i, 12);
        i += 12 + 8;
        for (j = 0; j < n && j < 31; j++)
        {
            /*des[j]=(char)rtcm_getbitu(rtcm->buff,i,8);*/ i += 8;
        }
        /*setup=rtcm_getbitu(rtcm->buff,i, 8);*/ i += 8 + 8;
        for (j = 0; j < m && j < 31; j++)
        {
            /*sno[j]=(char)rtcm_getbitu(rtcm->buff,i,8);*/ i += 8;
        }
    }
    else
    {
        trace(2, "rtcm3 1008 length error: len=%d\n", rtcm->len);
        return -1;
    }
    /* test station id */
    if (!test_staid(obs, staid))
        return -1;

    return 5;
}
/* decode type 1009-1012 message header --------------------------------------*/
static int decode_head1009(rtcm_t *rtcm, obs_t *obs, int *sync)
{
    double tod;
    char tstr[64];
    int i = 24, staid, nsat, type;

    type = rtcm_getbitu(rtcm->buff, i, 12);
    i += 12;

    if (i + 49 <= rtcm->len * 8)
    {
        staid = rtcm_getbitu(rtcm->buff, i, 12);
        i += 12;
        tod = rtcm_getbitu(rtcm->buff, i, 27) * 0.001;
        i += 27; /* sec in a day */
        *sync = rtcm_getbitu(rtcm->buff, i, 1);
        i += 1;
        nsat = rtcm_getbitu(rtcm->buff, i, 5);
    }
    else
    {
        trace(2, "rtcm3 %d length error: len=%d\n", type, rtcm->len);
        return -1;
    }
    /* test station id */
    if (!test_staid(obs, staid))
        return -1;

    adjday_glot(&rtcm->time, tod);

    time2str(rtcm->time, tstr, 2);
    trace(4, "decode_head1009: time=%s nsat=%d sync=%d\n", tstr, nsat, *sync);

    return nsat;
}
/* decode type 1009: L1-only glonass rtk observables -------------------------*/
static int decode_type1009(rtcm_t *rtcm, obs_t *obs)
{
    int sync;
    if (decode_head1009(rtcm, obs, &sync) < 0)
        return -1;
    obs->obsflag = !sync;
    return sync ? 0 : 1;
}
/* decode type 1010: extended L1-only glonass rtk observables ----------------*/
static int decode_type1010(rtcm_t *rtcm, obs_t *obs)
{
    double pr1, cnr1, tt, cp1, lam1;
    int i = 24 + 61, j, index, nsat, sync, prn, sat, code, freq, ppr1, lock1, amb, sys = _SYS_GLO_;

    if ((nsat = decode_head1009(rtcm, obs, &sync)) < 0)
        return -1;

    for (j = 0; j < nsat && obs->n < MAXOBS && i + 79 <= rtcm->len * 8; j++)
    {
        prn = rtcm_getbitu(rtcm->buff, i, 6);
        i += 6;
        code = rtcm_getbitu(rtcm->buff, i, 1);
        i += 1;
        freq = rtcm_getbitu(rtcm->buff, i, 5);
        i += 5;
        pr1 = rtcm_getbitu(rtcm->buff, i, 25);
        i += 25;
        ppr1 = rtcm_getbits(rtcm->buff, i, 20);
        i += 20;
        lock1 = rtcm_getbitu(rtcm->buff, i, 7);
        i += 7;
        amb = rtcm_getbitu(rtcm->buff, i, 7);
        i += 7;
        cnr1 = rtcm_getbitu(rtcm->buff, i, 8);
        i += 8;
        if (!(sat = satno(sys, prn)))
        {
            trace(2, "rtcm3 1010 satellite number error: prn=%d\n", prn);
            continue;
        }
        if ((index = obsindex(obs, rtcm->time, sat)) < 0)
            continue;
        pr1 = pr1 * 0.02 + amb * PRUNIT_GLO;
        if (ppr1 != (int)0xFFF80000)
        {
            obs->data[index].P[0] = pr1;
            lam1 = CLIGHT / (FREQ1_GLO + DFRQ1_GLO * (freq - 7));
            cp1 = adjcp(rtcm, sat, 0, ppr1 * 0.0005 / lam1);
            obs->data[index].L[0] = pr1 / lam1 + cp1;
        }
        obs->data[index].LLI[0] = (unsigned char)lossoflock(rtcm, sat, 0, lock1);
        obs->data[index].SNR[0] = snratio(cnr1 * 0.25);
        obs->data[index].code[0] = code ? CODE_L1P : CODE_L1C;
    }
    obs->obsflag = !sync;

    return sync ? 0 : 1;
}
/* decode type 1011: L1&L2 glonass rtk observables ---------------------------*/
static int decode_type1011(rtcm_t *rtcm, obs_t *obs)
{
    int sync;

    if (decode_head1009(rtcm, obs, &sync) < 0)
        return -1;
    obs->obsflag = !sync;

    return sync ? 0 : 1;
}
/* decode type 1012: extended L1&L2 glonass rtk observables ------------------*/
static int decode_type1012(rtcm_t *rtcm, obs_t *obs)
{
    double pr1, cnr1, cnr2, tt, cp1, cp2, lam1, lam2;
    int i = 24 + 61, j, index, nsat, sync, prn, sat, freq, code1, code2, pr21, ppr1, ppr2;
    int lock1, lock2, amb, sys = _SYS_GLO_;

    if ((nsat = decode_head1009(rtcm, obs, &sync)) < 0)
        return -1;

    for (j = 0; j < nsat && obs->n < MAXOBS && i + 130 <= rtcm->len * 8; j++)
    {
        prn = rtcm_getbitu(rtcm->buff, i, 6);
        i += 6;
        code1 = rtcm_getbitu(rtcm->buff, i, 1);
        i += 1;
        freq = rtcm_getbitu(rtcm->buff, i, 5);
        i += 5;
        pr1 = rtcm_getbitu(rtcm->buff, i, 25);
        i += 25;
        ppr1 = rtcm_getbits(rtcm->buff, i, 20);
        i += 20;
        lock1 = rtcm_getbitu(rtcm->buff, i, 7);
        i += 7;
        amb = rtcm_getbitu(rtcm->buff, i, 7);
        i += 7;
        cnr1 = rtcm_getbitu(rtcm->buff, i, 8);
        i += 8;
        code2 = rtcm_getbitu(rtcm->buff, i, 2);
        i += 2;
        pr21 = rtcm_getbits(rtcm->buff, i, 14);
        i += 14;
        ppr2 = rtcm_getbits(rtcm->buff, i, 20);
        i += 20;
        lock2 = rtcm_getbitu(rtcm->buff, i, 7);
        i += 7;
        cnr2 = rtcm_getbitu(rtcm->buff, i, 8);
        i += 8;
        if (!(sat = satno(sys, prn)))
        {
            trace(2, "rtcm3 1012 satellite number error: sys=%c prn=%d\n", sys2char(sys), prn);
            continue;
        }
        if ((index = obsindex(obs, rtcm->time, sat)) < 0)
            continue;
        pr1 = pr1 * 0.02 + amb * PRUNIT_GLO;
        if (ppr1 != (int)0xFFF80000)
        {
            lam1 = CLIGHT / (FREQ1_GLO + DFRQ1_GLO * (freq - 7));
            obs->data[index].P[0] = pr1;
            cp1 = adjcp(rtcm, sat, 0, ppr1 * 0.0005 / lam1);
            obs->data[index].L[0] = pr1 / lam1 + cp1;
        }
        obs->data[index].LLI[0] = (unsigned char)lossoflock(rtcm, sat, 0, lock1);
        obs->data[index].SNR[0] = snratio(cnr1 * 0.25);
        obs->data[index].code[0] = code1 ? CODE_L1P : CODE_L1C;

        if (pr21 != (int)0xFFFFE000)
        {
            obs->data[index].P[1] = pr1 + pr21 * 0.02;
        }
        if (ppr2 != (int)0xFFF80000)
        {
            lam2 = CLIGHT / (FREQ2_GLO + DFRQ2_GLO * (freq - 7));
            cp2 = adjcp(rtcm, sat, 1, ppr2 * 0.0005 / lam2);
            obs->data[index].L[1] = pr1 / lam2 + cp2;
        }
        obs->data[index].LLI[1] = (unsigned char)lossoflock(rtcm, sat, 1, lock2);
        obs->data[index].SNR[1] = snratio(cnr2 * 0.25);
        obs->data[index].code[1] = code2 ? CODE_L2P : CODE_L2C;
    }
    obs->obsflag = !sync;
    return sync ? 0 : 1;
}
/* decode type 1013: system parameters ---------------------------------------*/
static int decode_type1013(rtcm_t *rtcm)
{
    return 0;
}
/* decode type 1019: gps ephemerides -----------------------------------------*/
static int decode_type1019(rtcm_t *rtcm, nav_t *nav)
{
    eph_t eph = {0};
    double toc, sqrtA;
    int i = 24 + 12, prn, sat, week, sys = _SYS_GPS_;

    if (i + 476 <= rtcm->len * 8)
    {
        prn = rtcm_getbitu(rtcm->buff, i, 6);
        i += 6;
        week = rtcm_getbitu(rtcm->buff, i, 10);
        i += 10;
        eph.sva = rtcm_getbitu(rtcm->buff, i, 4);
        i += 4;
        eph.code = rtcm_getbitu(rtcm->buff, i, 2);
        i += 2;
        eph.idot = rtcm_getbits(rtcm->buff, i, 14) * P2_43 * SC2RAD;
        i += 14;
        eph.iode = rtcm_getbitu(rtcm->buff, i, 8);
        i += 8;
        toc = rtcm_getbitu(rtcm->buff, i, 16) * 16.0;
        i += 16;
        eph.f2 = rtcm_getbits(rtcm->buff, i, 8) * P2_55;
        i += 8;
        eph.f1 = rtcm_getbits(rtcm->buff, i, 16) * P2_43;
        i += 16;
        eph.f0 = rtcm_getbits(rtcm->buff, i, 22) * P2_31;
        i += 22;
        eph.iodc = rtcm_getbitu(rtcm->buff, i, 10);
        i += 10;
        eph.crs = rtcm_getbits(rtcm->buff, i, 16) * P2_5;
        i += 16;
        eph.deln = rtcm_getbits(rtcm->buff, i, 16) * P2_43 * SC2RAD;
        i += 16;
        eph.M0 = rtcm_getbits(rtcm->buff, i, 32) * P2_31 * SC2RAD;
        i += 32;
        eph.cuc = rtcm_getbits(rtcm->buff, i, 16) * P2_29;
        i += 16;
        eph.e = rtcm_getbitu(rtcm->buff, i, 32) * P2_33;
        i += 32;
        eph.cus = rtcm_getbits(rtcm->buff, i, 16) * P2_29;
        i += 16;
        sqrtA = rtcm_getbitu(rtcm->buff, i, 32) * P2_19;
        i += 32;
        eph.toes = rtcm_getbitu(rtcm->buff, i, 16) * 16.0;
        i += 16;
        eph.cic = rtcm_getbits(rtcm->buff, i, 16) * P2_29;
        i += 16;
        eph.OMG0 = rtcm_getbits(rtcm->buff, i, 32) * P2_31 * SC2RAD;
        i += 32;
        eph.cis = rtcm_getbits(rtcm->buff, i, 16) * P2_29;
        i += 16;
        eph.i0 = rtcm_getbits(rtcm->buff, i, 32) * P2_31 * SC2RAD;
        i += 32;
        eph.crc = rtcm_getbits(rtcm->buff, i, 16) * P2_5;
        i += 16;
        eph.omg = rtcm_getbits(rtcm->buff, i, 32) * P2_31 * SC2RAD;
        i += 32;
        eph.OMGd = rtcm_getbits(rtcm->buff, i, 24) * P2_43 * SC2RAD;
        i += 24;
        eph.tgd[0] = rtcm_getbits(rtcm->buff, i, 8) * P2_31;
        i += 8;
        eph.svh = rtcm_getbitu(rtcm->buff, i, 6);
        i += 6;
        eph.flag = rtcm_getbitu(rtcm->buff, i, 1);
        i += 1;
        eph.fit = rtcm_getbitu(rtcm->buff, i, 1) ? 0.0 : 4.0; /* 0:4hr,1:>4hr */
    }
    else
    {
        trace(2, "rtcm3 1019 length error: len=%d\n", rtcm->len);
        return -1;
    }
    if (prn >= 40)
    {
        sys = _SYS_SBS_;
        prn += 80;
    }
    trace(4, "decode_type1019: prn=%c%02d iode=%d toe=%.0f\n", sys2char(sys), prn, eph.iode, eph.toes);

    if (!(sat = satno(sys, prn)))
    {
        trace(2, "rtcm3 1019 satellite number error: prn=%c%02d\n", sys2char(sys), prn);
        return -1;
    }
    eph.sat = sat;
    eph.week = adjgpsweek(&rtcm->time, week);
    eph.toe = gpst2time(eph.week, eph.toes);
    eph.toc = gpst2time(eph.week, toc);
    eph.ttr = rtcm->time;
    eph.A = sqrtA * sqrtA;

   	if (add_eph(&eph, nav)==1)
		++nav->n_gps;

    return 2;
}
/* decode type 1020: glonass ephemerides -------------------------------------*/
static int decode_type1020(rtcm_t *rtcm, nav_t *nav)
{
    geph_t geph = {0};
    double tk_h, tk_m, tk_s, toe, tow, tod, tof;
    int i = 24 + 12, prn, sat, week, tb, bn, sys = _SYS_GLO_;

    if (i + 348 <= rtcm->len * 8)
    {
        prn = rtcm_getbitu(rtcm->buff, i, 6);
        i += 6;
        geph.frq = rtcm_getbitu(rtcm->buff, i, 5) - 7;
        i += 5 + 2 + 2;
        tk_h = rtcm_getbitu(rtcm->buff, i, 5);
        i += 5;
        tk_m = rtcm_getbitu(rtcm->buff, i, 6);
        i += 6;
        tk_s = rtcm_getbitu(rtcm->buff, i, 1) * 30.0;
        i += 1;
        bn = rtcm_getbitu(rtcm->buff, i, 1);
        i += 1 + 1;
        tb = rtcm_getbitu(rtcm->buff, i, 7);
        i += 7;
        geph.vel[0] = getbitg(rtcm->buff, i, 24) * P2_20 * 1E3;
        i += 24;
        geph.pos[0] = getbitg(rtcm->buff, i, 27) * P2_11 * 1E3;
        i += 27;
        geph.acc[0] = getbitg(rtcm->buff, i, 5) * P2_30 * 1E3;
        i += 5;
        geph.vel[1] = getbitg(rtcm->buff, i, 24) * P2_20 * 1E3;
        i += 24;
        geph.pos[1] = getbitg(rtcm->buff, i, 27) * P2_11 * 1E3;
        i += 27;
        geph.acc[1] = getbitg(rtcm->buff, i, 5) * P2_30 * 1E3;
        i += 5;
        geph.vel[2] = getbitg(rtcm->buff, i, 24) * P2_20 * 1E3;
        i += 24;
        geph.pos[2] = getbitg(rtcm->buff, i, 27) * P2_11 * 1E3;
        i += 27;
        geph.acc[2] = getbitg(rtcm->buff, i, 5) * P2_30 * 1E3;
        i += 5 + 1;
        geph.gamn = getbitg(rtcm->buff, i, 11) * P2_40;
        i += 11 + 3;
        geph.taun = getbitg(rtcm->buff, i, 22) * P2_30;

        set_glo_frq(prn, geph.frq);
    }
    else
    {
        trace(2, "rtcm3 1020 length error: len=%d\n", rtcm->len);
        return -1;
    }
    if (!(sat = satno(sys, prn)))
    {
        trace(2, "rtcm3 1020 satellite number error: prn=%c%02d\n", sys2char(sys), prn);
        return -1;
    }
    trace(4, "decode_type1020: prn=%c%02d tk=%02.0f:%02.0f:%02.0f\n", sys2char(sys), prn, tk_h, tk_m, tk_s);

    geph.sat = sat;
    geph.svh = bn;
    geph.iode = tb & 0x7F;
    if (rtcm->time.time == 0)
        rtcm->time = utc2gpst(timeget());
    tow = time2gpst(gpst2utc(rtcm->time), &week);
    tod = fmod(tow, 86400.0);
    tow -= tod;
    tof = tk_h * 3600.0 + tk_m * 60.0 + tk_s - 10800.0; /* lt->utc */
    if (tof < tod - 43200.0)
        tof += 86400.0;
    else if (tof > tod + 43200.0)
        tof -= 86400.0;
    geph.tof = utc2gpst(gpst2time(week, tow + tof));
    toe = tb * 900.0 - 10800.0; /* lt->utc */
    if (toe < tod - 43200.0)
        toe += 86400.0;
    else if (toe > tod + 43200.0)
        toe -= 86400.0;
    geph.toe = utc2gpst(gpst2time(week, tow + toe)); /* utc->gpst */

    add_geph(&geph, nav);
    
    return 2;
}
/* decode type 1021: helmert/abridged molodenski -----------------------------*/
static int decode_type1021(rtcm_t *rtcm)
{
    trace(2, "rtcm3 1021: not supported message\n");
    return 0;
}
/* decode type 1022: moledenski-badekas transfromation -----------------------*/
static int decode_type1022(rtcm_t *rtcm)
{
    trace(2, "rtcm3 1022: not supported message\n");
    return 0;
}
/* decode type 1023: residual, ellipoidal grid representation ----------------*/
static int decode_type1023(rtcm_t *rtcm)
{
    trace(2, "rtcm3 1023: not supported message\n");
    return 0;
}
/* decode type 1024: residual, plane grid representation ---------------------*/
static int decode_type1024(rtcm_t *rtcm)
{
    trace(2, "rtcm3 1024: not supported message\n");
    return 0;
}
/* decode type 1025: projection (types except LCC2SP,OM) ---------------------*/
static int decode_type1025(rtcm_t *rtcm)
{
    trace(2, "rtcm3 1025: not supported message\n");
    return 0;
}
/* decode type 1026: projection (LCC2SP - lambert conic conformal (2sp)) -----*/
static int decode_type1026(rtcm_t *rtcm)
{
    trace(2, "rtcm3 1026: not supported message\n");
    return 0;
}
/* decode type 1027: projection (type OM - oblique mercator) -----------------*/
static int decode_type1027(rtcm_t *rtcm)
{
    trace(2, "rtcm3 1027: not supported message\n");
    return 0;
}
/* decode type 1029: unicode text string -------------------------------------*/
static int decode_type1029(rtcm_t *rtcm)
{
    int i = 24 + 12, staid, mjd, tod, nchar, cunit;

    if (i + 60 <= rtcm->len * 8)
    {
        staid = rtcm_getbitu(rtcm->buff, i, 12);
        i += 12;
        mjd = rtcm_getbitu(rtcm->buff, i, 16);
        i += 16;
        tod = rtcm_getbitu(rtcm->buff, i, 17);
        i += 17;
        nchar = rtcm_getbitu(rtcm->buff, i, 7);
        i += 7;
        cunit = rtcm_getbitu(rtcm->buff, i, 8);
        i += 8;
    }
    else
    {
        trace(2, "rtcm3 1029 length error: len=%d\n", rtcm->len);
        return -1;
    }
    if (i + nchar * 8 > rtcm->len * 8)
    {
        trace(2, "rtcm3 1029 length error: len=%d nchar=%d\n", rtcm->len, nchar);
        return -1;
    }

    return 0;
}
/* decode type 1030: network rtk residual ------------------------------------*/
static int decode_type1030(rtcm_t *rtcm)
{
    trace(2, "rtcm3 1030: not supported message\n");
    return 0;
}
/* decode type 1031: glonass network rtk residual ----------------------------*/
static int decode_type1031(rtcm_t *rtcm)
{
    trace(2, "rtcm3 1031: not supported message\n");
    return 0;
}
/* decode type 1032: physical reference station position information ---------*/
static int decode_type1032(rtcm_t *rtcm)
{
    trace(2, "rtcm3 1032: not supported message\n");
    return 0;
}
/* decode type 1033: receiver and antenna descriptor -------------------------*/
static int decode_type1033(rtcm_t *rtcm, obs_t *obs)
{
    char des[32] = "", sno[32] = "", rec[32] = "", ver[32] = "", rsn[32] = "";
    int i = 24 + 12, j, staid, n, m, n1, n2, n3, setup;

    n = rtcm_getbitu(rtcm->buff, i + 12, 8);
    m = rtcm_getbitu(rtcm->buff, i + 28 + 8 * n, 8);
    n1 = rtcm_getbitu(rtcm->buff, i + 36 + 8 * (n + m), 8);
    n2 = rtcm_getbitu(rtcm->buff, i + 44 + 8 * (n + m + n1), 8);
    n3 = rtcm_getbitu(rtcm->buff, i + 52 + 8 * (n + m + n1 + n2), 8);

    if (i + 60 + 8 * (n + m + n1 + n2 + n3) <= rtcm->len * 8)
    {
        staid = rtcm_getbitu(rtcm->buff, i, 12);
        i += 12 + 8;
        for (j = 0; j < n && j < 31; j++)
        {
            des[j] = (char)rtcm_getbitu(rtcm->buff, i, 8);
            i += 8;
        }
        setup = rtcm_getbitu(rtcm->buff, i, 8);
        i += 8 + 8;
        for (j = 0; j < m && j < 31; j++)
        {
            sno[j] = (char)rtcm_getbitu(rtcm->buff, i, 8);
            i += 8;
        }
        i += 8;
        for (j = 0; j < n1 && j < 31; j++)
        {
            rec[j] = (char)rtcm_getbitu(rtcm->buff, i, 8);
            i += 8;
        }
        i += 8;
        for (j = 0; j < n2 && j < 31; j++)
        {
            ver[j] = (char)rtcm_getbitu(rtcm->buff, i, 8);
            i += 8;
        }
        i += 8;
        for (j = 0; j < n3 && j < 31; j++)
        {
            rsn[j] = (char)rtcm_getbitu(rtcm->buff, i, 8);
            i += 8;
        }
    }
    else
    {
        trace(2, "rtcm3 1033 length error: len=%d\n", rtcm->len);
        return -1;
    }
    /* test station id */
    if (!test_staid(obs, staid))
        return -1;

    trace(3, "rtcm3 1033: ant=%s:%s rec=%s:%s:%s\n", des, sno, rec, ver, rsn);

    return 5;
}
/* decode type 1034: gps network fkp gradient --------------------------------*/
static int decode_type1034(rtcm_t *rtcm)
{
    trace(2, "rtcm3 1034: not supported message\n");
    return 0;
}
/* decode type 1035: glonass network fkp gradient ----------------------------*/
static int decode_type1035(rtcm_t *rtcm)
{
    trace(2, "rtcm3 1035: not supported message\n");
    return 0;
}
/* decode type 1037: glonass network rtk ionospheric correction difference ---*/
static int decode_type1037(rtcm_t *rtcm)
{
    int i = 0;
    trace(2, "rtcm3 1037: not supported message\n");
    return 0;
}
/* decode type 1038: glonass network rtk geometic correction difference ------*/
static int decode_type1038(rtcm_t *rtcm)
{
    trace(2, "rtcm3 1038: not supported message\n");
    return 0;
}
/* decode type 1039: glonass network rtk combined correction difference ------*/
static int decode_type1039(rtcm_t *rtcm)
{
    trace(2, "rtcm3 1039: not supported message\n");
    return 0;
}
/* decode type 1044: qzss ephemerides (ref [15]) -----------------------------*/
static int decode_type1044(rtcm_t *rtcm, nav_t *nav)
{
    eph_t eph = {0};
    double toc, sqrtA;
    int i = 24 + 12, prn, sat, week, sys = _SYS_QZS_;

    if (i + 473 <= rtcm->len * 8)
    {
        prn = rtcm_getbitu(rtcm->buff, i, 4) + 192;
        i += 4;
        toc = rtcm_getbitu(rtcm->buff, i, 16) * 16.0;
        i += 16;
        eph.f2 = rtcm_getbits(rtcm->buff, i, 8) * P2_55;
        i += 8;
        eph.f1 = rtcm_getbits(rtcm->buff, i, 16) * P2_43;
        i += 16;
        eph.f0 = rtcm_getbits(rtcm->buff, i, 22) * P2_31;
        i += 22;
        eph.iode = rtcm_getbitu(rtcm->buff, i, 8);
        i += 8;
        eph.crs = rtcm_getbits(rtcm->buff, i, 16) * P2_5;
        i += 16;
        eph.deln = rtcm_getbits(rtcm->buff, i, 16) * P2_43 * SC2RAD;
        i += 16;
        eph.M0 = rtcm_getbits(rtcm->buff, i, 32) * P2_31 * SC2RAD;
        i += 32;
        eph.cuc = rtcm_getbits(rtcm->buff, i, 16) * P2_29;
        i += 16;
        eph.e = rtcm_getbitu(rtcm->buff, i, 32) * P2_33;
        i += 32;
        eph.cus = rtcm_getbits(rtcm->buff, i, 16) * P2_29;
        i += 16;
        sqrtA = rtcm_getbitu(rtcm->buff, i, 32) * P2_19;
        i += 32;
        eph.toes = rtcm_getbitu(rtcm->buff, i, 16) * 16.0;
        i += 16;
        eph.cic = rtcm_getbits(rtcm->buff, i, 16) * P2_29;
        i += 16;
        eph.OMG0 = rtcm_getbits(rtcm->buff, i, 32) * P2_31 * SC2RAD;
        i += 32;
        eph.cis = rtcm_getbits(rtcm->buff, i, 16) * P2_29;
        i += 16;
        eph.i0 = rtcm_getbits(rtcm->buff, i, 32) * P2_31 * SC2RAD;
        i += 32;
        eph.crc = rtcm_getbits(rtcm->buff, i, 16) * P2_5;
        i += 16;
        eph.omg = rtcm_getbits(rtcm->buff, i, 32) * P2_31 * SC2RAD;
        i += 32;
        eph.OMGd = rtcm_getbits(rtcm->buff, i, 24) * P2_43 * SC2RAD;
        i += 24;
        eph.idot = rtcm_getbits(rtcm->buff, i, 14) * P2_43 * SC2RAD;
        i += 14;
        eph.code = rtcm_getbitu(rtcm->buff, i, 2);
        i += 2;
        week = rtcm_getbitu(rtcm->buff, i, 10);
        i += 10;
        eph.sva = rtcm_getbitu(rtcm->buff, i, 4);
        i += 4;
        eph.svh = rtcm_getbitu(rtcm->buff, i, 6);
        i += 6;
        eph.tgd[0] = rtcm_getbits(rtcm->buff, i, 8) * P2_31;
        i += 8;
        eph.iodc = rtcm_getbitu(rtcm->buff, i, 10);
        i += 10;
        eph.fit = rtcm_getbitu(rtcm->buff, i, 1) ? 0.0 : 2.0; /* 0:2hr,1:>2hr */
    }
    else
    {
        trace(2, "rtcm3 1044 length error: len=%d\n", rtcm->len);
        return -1;
    }
    trace(4, "decode_type1044: prn=%c%02d iode=%d toe=%.0f\n", sys2char(sys), prn, eph.iode, eph.toes);

    if (!(sat = satno(sys, prn)))
    {
        trace(2, "rtcm3 1044 satellite number error: prn=%c%02d\n", sys2char(sys), prn);
        return -1;
    }
    eph.sat = sat;
    eph.week = adjgpsweek(&rtcm->time, week);
    eph.toe = gpst2time(eph.week, eph.toes);
    eph.toc = gpst2time(eph.week, toc);
    eph.ttr = rtcm->time;
    eph.A = sqrtA * sqrtA;
    
    /* do not use QZSS now */
#ifdef ENAQZS
	if (add_eph(&eph, nav) == 1)
		++nav->n_qzs;
#endif

    return 2;
}
/* decode type 1045: galileo F/NAV satellite ephemerides (ref [15]) ----------*/
static int decode_type1045(rtcm_t *rtcm, nav_t *nav)
{
    eph_t eph = {0};
    double toc, sqrtA, ws;
    int i = 24 + 12, prn, sat, week, e5a_hs, e5a_dvs, rsv, sys = _SYS_GAL_, wk;

    if (i + 484 <= rtcm->len * 8)
    {
        prn = rtcm_getbitu(rtcm->buff, i, 6);
        i += 6;
        week = rtcm_getbitu(rtcm->buff, i, 12);
        i += 12; /* gst-week */
        eph.iode = rtcm_getbitu(rtcm->buff, i, 10);
        i += 10;
        eph.sva = rtcm_getbitu(rtcm->buff, i, 8);
        i += 8;
        eph.idot = rtcm_getbits(rtcm->buff, i, 14) * P2_43 * SC2RAD;
        i += 14;
        toc = rtcm_getbitu(rtcm->buff, i, 14) * 60.0;
        i += 14;
        eph.f2 = rtcm_getbits(rtcm->buff, i, 6) * P2_59;
        i += 6;
        eph.f1 = rtcm_getbits(rtcm->buff, i, 21) * P2_46;
        i += 21;
        eph.f0 = rtcm_getbits(rtcm->buff, i, 31) * P2_34;
        i += 31;
        eph.crs = rtcm_getbits(rtcm->buff, i, 16) * P2_5;
        i += 16;
        eph.deln = rtcm_getbits(rtcm->buff, i, 16) * P2_43 * SC2RAD;
        i += 16;
        eph.M0 = rtcm_getbits(rtcm->buff, i, 32) * P2_31 * SC2RAD;
        i += 32;
        eph.cuc = rtcm_getbits(rtcm->buff, i, 16) * P2_29;
        i += 16;
        eph.e = rtcm_getbitu(rtcm->buff, i, 32) * P2_33;
        i += 32;
        eph.cus = rtcm_getbits(rtcm->buff, i, 16) * P2_29;
        i += 16;
        sqrtA = rtcm_getbitu(rtcm->buff, i, 32) * P2_19;
        i += 32;
        eph.toes = rtcm_getbitu(rtcm->buff, i, 14) * 60.0;
        i += 14;
        eph.cic = rtcm_getbits(rtcm->buff, i, 16) * P2_29;
        i += 16;
        eph.OMG0 = rtcm_getbits(rtcm->buff, i, 32) * P2_31 * SC2RAD;
        i += 32;
        eph.cis = rtcm_getbits(rtcm->buff, i, 16) * P2_29;
        i += 16;
        eph.i0 = rtcm_getbits(rtcm->buff, i, 32) * P2_31 * SC2RAD;
        i += 32;
        eph.crc = rtcm_getbits(rtcm->buff, i, 16) * P2_5;
        i += 16;
        eph.omg = rtcm_getbits(rtcm->buff, i, 32) * P2_31 * SC2RAD;
        i += 32;
        eph.OMGd = rtcm_getbits(rtcm->buff, i, 24) * P2_43 * SC2RAD;
        i += 24;
        eph.tgd[0] = rtcm_getbits(rtcm->buff, i, 10) * P2_32;
        i += 10; /* E5a/E1 */
        e5a_hs = rtcm_getbitu(rtcm->buff, i, 2);
        i += 2; /* OSHS */
        e5a_dvs = rtcm_getbitu(rtcm->buff, i, 1);
        i += 1; /* OSDVS */
        rsv = rtcm_getbitu(rtcm->buff, i, 7);
    }
    else
    {
        trace(2, "rtcm3 1045 length error: len=%d\n", rtcm->len);
        return -1;
    }
    trace(4, "decode_type1045: prn=%c%02d iode=%d toe=%.0f\n", sys2char(sys), prn, eph.iode, eph.toes);

    if (!(sat = satno(sys, prn)))
    {
        trace(2, "rtcm3 1045 satellite number error: prn=%c%02d\n", sys2char(sys), prn);
        return -1;
    }
    eph.sat = sat;
    eph.week = week + 1024; /* gal-week = gst-week + 1024 */
    set_week_number(eph.week);
    ws = time2gpst(rtcm->time, &wk);
    if (wk != eph.week)
        rtcm->time = gpst2time(eph.week, ws);
    eph.toe = gpst2time(eph.week, eph.toes);
    eph.toc = gpst2time(eph.week, toc);
    eph.ttr = rtcm->time;
    eph.A = sqrtA * sqrtA;
    eph.svh = (e5a_hs << 4) + (e5a_dvs << 3);
    eph.code = (1 << 1) | (1 << 8); /* data source = f/nav e5a + af0-2,toc,sisa for e5a-e1 */
    
    if (add_eph(&eph, nav) == 1)
		++nav->n_gal;

    return 2;
}
/* decode type 1046: galileo I/NAV satellite ephemerides (ref [17]) ----------*/
static int decode_type1046(rtcm_t *rtcm, nav_t *nav)
{
    eph_t eph = {0};
    double toc, sqrtA, ws;
    int i = 24 + 12, prn, sat, week, e5b_hs, e5b_dvs, e1_hs, e1_dvs, sys = _SYS_GAL_, wk;

    if (i + 492 <= rtcm->len * 8)
    {
        prn = rtcm_getbitu(rtcm->buff, i, 6);
        i += 6;
        week = rtcm_getbitu(rtcm->buff, i, 12);
        i += 12;
        eph.iode = rtcm_getbitu(rtcm->buff, i, 10);
        i += 10;
        eph.sva = rtcm_getbitu(rtcm->buff, i, 8);
        i += 8;
        eph.idot = rtcm_getbits(rtcm->buff, i, 14) * P2_43 * SC2RAD;
        i += 14;
        toc = rtcm_getbitu(rtcm->buff, i, 14) * 60.0;
        i += 14;
        eph.f2 = rtcm_getbits(rtcm->buff, i, 6) * P2_59;
        i += 6;
        eph.f1 = rtcm_getbits(rtcm->buff, i, 21) * P2_46;
        i += 21;
        eph.f0 = rtcm_getbits(rtcm->buff, i, 31) * P2_34;
        i += 31;
        eph.crs = rtcm_getbits(rtcm->buff, i, 16) * P2_5;
        i += 16;
        eph.deln = rtcm_getbits(rtcm->buff, i, 16) * P2_43 * SC2RAD;
        i += 16;
        eph.M0 = rtcm_getbits(rtcm->buff, i, 32) * P2_31 * SC2RAD;
        i += 32;
        eph.cuc = rtcm_getbits(rtcm->buff, i, 16) * P2_29;
        i += 16;
        eph.e = rtcm_getbitu(rtcm->buff, i, 32) * P2_33;
        i += 32;
        eph.cus = rtcm_getbits(rtcm->buff, i, 16) * P2_29;
        i += 16;
        sqrtA = rtcm_getbitu(rtcm->buff, i, 32) * P2_19;
        i += 32;
        eph.toes = rtcm_getbitu(rtcm->buff, i, 14) * 60.0;
        i += 14;
        eph.cic = rtcm_getbits(rtcm->buff, i, 16) * P2_29;
        i += 16;
        eph.OMG0 = rtcm_getbits(rtcm->buff, i, 32) * P2_31 * SC2RAD;
        i += 32;
        eph.cis = rtcm_getbits(rtcm->buff, i, 16) * P2_29;
        i += 16;
        eph.i0 = rtcm_getbits(rtcm->buff, i, 32) * P2_31 * SC2RAD;
        i += 32;
        eph.crc = rtcm_getbits(rtcm->buff, i, 16) * P2_5;
        i += 16;
        eph.omg = rtcm_getbits(rtcm->buff, i, 32) * P2_31 * SC2RAD;
        i += 32;
        eph.OMGd = rtcm_getbits(rtcm->buff, i, 24) * P2_43 * SC2RAD;
        i += 24;
        eph.tgd[0] = rtcm_getbits(rtcm->buff, i, 10) * P2_32;
        i += 10; /* E5a/E1 */
        eph.tgd[1] = rtcm_getbits(rtcm->buff, i, 10) * P2_32;
        i += 10; /* E5b/E1 */
        e5b_hs = rtcm_getbitu(rtcm->buff, i, 2);
        i += 2; /* E5b OSHS */
        e5b_dvs = rtcm_getbitu(rtcm->buff, i, 1);
        i += 1; /* E5b OSDVS */
        e1_hs = rtcm_getbitu(rtcm->buff, i, 2);
        i += 2; /* E1 OSHS */
        e1_dvs = rtcm_getbitu(rtcm->buff, i, 1);
        i += 1; /* E1 OSDVS */
    }
    else
    {
        trace(2, "rtcm3 1046 length error: len=%d\n", rtcm->len);
        return -1;
    }
    trace(4, "decode_type1046: prn=%c%02d iode=%d toe=%.0f\n", sys2char(sys), prn, eph.iode, eph.toes);

    if (!(sat = satno(sys, prn)))
    {
        trace(2, "rtcm3 1046 satellite number error: prn=%c%02d\n", sys2char(sys), prn);
        return -1;
    }

    eph.sat = sat;
    eph.week = week + 1024; /* gal-week = gst-week + 1024 */
    set_week_number(eph.week);
    ws = time2gpst(rtcm->time, &wk);
    if (wk != eph.week)
        rtcm->time = gpst2time(eph.week, ws);
    eph.toe = gpst2time(eph.week, eph.toes);
    eph.toc = gpst2time(eph.week, toc);
    eph.ttr = rtcm->time;
    eph.A = sqrtA * sqrtA;
    eph.svh = (e5b_hs << 7) + (e5b_dvs << 6) + (e1_hs << 1) + (e1_dvs << 0);
    eph.code = (1 << 0) | (1 << 9); /* data source = i/nav e1b + af0-2,toc,sisa for e5b-e1 */

    if (add_eph(&eph, nav)==1)
		++nav->n_gal;
    
    return 2;
}
/* decode type 1042/63: beidou ephemerides -----------------------------------*/
static int decode_type1042(rtcm_t *rtcm, nav_t *nav)
{
    eph_t eph = {0};
    double toc, sqrtA, ws;
    int i = 24 + 12, prn, sat, week, sys = _SYS_BDS_, wk;

    if (i + 499 <= rtcm->len * 8)
    {
        prn = rtcm_getbitu(rtcm->buff, i, 6);
        i += 6;
        week = rtcm_getbitu(rtcm->buff, i, 13);
        i += 13;
        eph.sva = rtcm_getbitu(rtcm->buff, i, 4);
        i += 4;
        eph.idot = rtcm_getbits(rtcm->buff, i, 14) * P2_43 * SC2RAD;
        i += 14;
        eph.iode = rtcm_getbitu(rtcm->buff, i, 5);
        i += 5; /* AODE */
        toc = rtcm_getbitu(rtcm->buff, i, 17) * 8.0;
        i += 17;
        eph.f2 = rtcm_getbits(rtcm->buff, i, 11) * P2_66;
        i += 11;
        eph.f1 = rtcm_getbits(rtcm->buff, i, 22) * P2_50;
        i += 22;
        eph.f0 = rtcm_getbits(rtcm->buff, i, 24) * P2_33;
        i += 24;
        eph.iodc = rtcm_getbitu(rtcm->buff, i, 5);
        i += 5; /* AODC */
        eph.crs = rtcm_getbits(rtcm->buff, i, 18) * P2_6;
        i += 18;
        eph.deln = rtcm_getbits(rtcm->buff, i, 16) * P2_43 * SC2RAD;
        i += 16;
        eph.M0 = rtcm_getbits(rtcm->buff, i, 32) * P2_31 * SC2RAD;
        i += 32;
        eph.cuc = rtcm_getbits(rtcm->buff, i, 18) * P2_31;
        i += 18;
        eph.e = rtcm_getbitu(rtcm->buff, i, 32) * P2_33;
        i += 32;
        eph.cus = rtcm_getbits(rtcm->buff, i, 18) * P2_31;
        i += 18;
        sqrtA = rtcm_getbitu(rtcm->buff, i, 32) * P2_19;
        i += 32;
        eph.toes = rtcm_getbitu(rtcm->buff, i, 17) * 8.0;
        i += 17;
        eph.cic = rtcm_getbits(rtcm->buff, i, 18) * P2_31;
        i += 18;
        eph.OMG0 = rtcm_getbits(rtcm->buff, i, 32) * P2_31 * SC2RAD;
        i += 32;
        eph.cis = rtcm_getbits(rtcm->buff, i, 18) * P2_31;
        i += 18;
        eph.i0 = rtcm_getbits(rtcm->buff, i, 32) * P2_31 * SC2RAD;
        i += 32;
        eph.crc = rtcm_getbits(rtcm->buff, i, 18) * P2_6;
        i += 18;
        eph.omg = rtcm_getbits(rtcm->buff, i, 32) * P2_31 * SC2RAD;
        i += 32;
        eph.OMGd = rtcm_getbits(rtcm->buff, i, 24) * P2_43 * SC2RAD;
        i += 24;
        eph.tgd[0] = rtcm_getbits(rtcm->buff, i, 10) * 1E-10;
        i += 10;
        eph.tgd[1] = rtcm_getbits(rtcm->buff, i, 10) * 1E-10;
        i += 10;
        eph.svh = rtcm_getbitu(rtcm->buff, i, 1);
        i += 1;
    }
    else
    {
        trace(2, "rtcm3 1042 length error: len=%d\n", rtcm->len);
        return -1;
    }
    trace(4, "decode_type1042: prn=%c%02d iode=%d toe=%.0f\n", sys2char(sys), prn, eph.iode, eph.toes);

    if (!(sat = satno(sys, prn)))
    {
        trace(2, "rtcm3 1042 satellite number error: prn=%c%02d\n", sys2char(sys), prn);
        return -1;
    }
    eph.sat = sat;
    eph.week = adjbdtweek(&rtcm->time, week); // 1356 + week; //
    set_week_number(eph.week + 1356);
    // ws = time2gpst(rtcm->time, &wk);
    // if (wk != eph.week)
    //     rtcm->time = gpst2time(eph.week, ws);
    eph.toe = bdt2gpst(bdt2time(eph.week, eph.toes)); /* bdt -> gpst */
    eph.toc = bdt2gpst(bdt2time(eph.week, toc));      /* bdt -> gpst */
    eph.ttr = rtcm->time;
    eph.A = sqrtA * sqrtA;

    if (add_eph(&eph, nav) == 1)
		++nav->n_bds;

    return 2;
}
/* decode type 4001 -------------------------------------------------*/
static int decode_type4001(rtcm_t *rtcm)
{
	double ep[6] = { 0 };
	int i = 24 + 12;
	ep[0] = (double)rtcm_getbitu(rtcm->buff, i, 12);
	i += 12;
	ep[1] = (double)rtcm_getbitu(rtcm->buff, i, 4);
	i += 4;
	ep[2] = (double)rtcm_getbitu(rtcm->buff, i, 6);
	i += 6;
	ep[3] = (double)rtcm_getbitu(rtcm->buff, i, 8);
	i += 8;
	ep[4] = (double)rtcm_getbitu(rtcm->buff, i, 8);
	i += 8;
	ep[5] = (double)rtcm_getbitu(rtcm->buff, i, 8);
	i += 8;

#ifdef ARM_MCU
	rtcm->time = epoch2time(ep);
#endif    
	// printf("%d %d %d %d %d %d\n", ep[0], ep[1], ep[2], ep[3], ep[4], ep[5]);
#ifdef INT_SEC_SEND
	sensor_time_s.year = (uint32_t)(ep[0]);
	sensor_time_s.month = (uint32_t)(ep[1]);
	sensor_time_s.date = (uint32_t)(ep[2]);
	sensor_time_s.hour = (uint32_t)(ep[3]);
	sensor_time_s.minute = (uint32_t)(ep[4]);
	sensor_time_s.second = (uint32_t)(ep[5]);
	sensor_time_s.ms = 0;
#endif
	return i;
}

/* save obs data in msm message ----------------------------------------------*/
static void save_msm_obs(rtcm_t *rtcm, obs_t *obs, int sys, msm_h_t *h, const double *r,
                         const double *pr, const double *cp, const double *rr,
                         const double *rrf, const double *cnr, const int *lock,
                         const int *ex, const int *half)
{
    const char *sig[32];
    double tt, wl;
    unsigned char code[32];
    char *msm_type = "", *q = NULL;
    int i, j, k, type, prn, sat, fn, index = 0, freq[32], ind[32];
    char opt[256] = {0};
	obsd_t obsd = { 0 };

    type = rtcm_getbitu(rtcm->buff, 24, 12);

    switch (sys)
    {
    case _SYS_GPS_:
        msm_type = q = rtcm->msmtype[0];
        break;
    case _SYS_GLO_:
        msm_type = q = rtcm->msmtype[1];
        break;
    case _SYS_GAL_:
        msm_type = q = rtcm->msmtype[2];
        break;
    case _SYS_QZS_:
        msm_type = q = rtcm->msmtype[3];
        break;
    case _SYS_SBS_:
        msm_type = q = rtcm->msmtype[4];
        break;
    case _SYS_BDS_:
        msm_type = q = rtcm->msmtype[5];
        break;
    }
    /* id to signal */
    for (i = 0; i < h->nsig; i++)
    {
        switch (sys)
        {
        case _SYS_GPS_:
            sig[i] = rtcm_msm_sig_gps[h->sigs[i] - 1];
            break;
        case _SYS_GLO_:
            sig[i] = rtcm_msm_sig_glo[h->sigs[i] - 1];
            break;
        case _SYS_GAL_:
            sig[i] = msm_sig_gal[h->sigs[i] - 1];
            break;
        case _SYS_QZS_:
            sig[i] = msm_sig_qzs[h->sigs[i] - 1];
            break;
        case _SYS_SBS_:
            sig[i] = msm_sig_sbs[h->sigs[i] - 1];
            break;
        case _SYS_BDS_:
            sig[i] = msm_sig_cmp[h->sigs[i] - 1];
            break;
        default:
            sig[i] = "";
            break;
        }
        /* signal to rinex obs type */
        code[i] = obs2code(sys, sig[i], freq + i);

        if (code[i] != CODE_NONE)
        {
            if (q)
                q += sprintf(q, "L%s%s", sig[i], i < h->nsig - 1 ? "," : "");
        }
        else
        {
            if (q)
                q += sprintf(q, "(%d)%s", h->sigs[i], i < h->nsig - 1 ? "," : "");

            trace(2, "rtcm3 %d: unknown signal id=%2d\n", type, h->sigs[i]);
        }
    }
    trace(3, "rtcm3 %d: signals=%s\n", type, msm_type);

    /* get signal index */
    sigindex(sys, code, freq, h->nsig, opt, ind);

    for (i = j = 0; i < h->nsat; i++)
    {

        prn = h->sats[i];
        if (sys == _SYS_QZS_)
            prn += MINPRNQZS - 1;
        else if (sys == _SYS_SBS_)
            prn += MINPRNSBS - 1;

		memset(&obsd, 0, sizeof(obsd_t));
        if ((sat = satno(sys, prn)))
        {
            //index = obsindex(obs, rtcm->time, sat);
			obsd.sat = sat;
			obsd.time = rtcm->time;
        }
        else
        {
            trace(2, "rtcm3 %d satellite error: prn=%c%02d\n", type, sys2char(sys), prn);
        }
        for (k = 0; k < h->nsig; k++)
        {
            if (!h->cellmask[k + i * h->nsig])
                continue;

			//if (sat && index>=0 && ind[k] >= 0)
            if (sat && ind[k] >= 0)
            {

                /* satellite carrier wave length */
                wl = satwavelen(sat, freq[k] - 1);

                /* glonass wave length by extended info */
                if (sys == _SYS_GLO_ && ex && ex[i] <= 13)
                {
                    fn = ex[i] - 7;
                    wl = CLIGHT / ((freq[k] == 2 ? FREQ2_GLO : FREQ1_GLO) +
                                   (freq[k] == 2 ? DFRQ2_GLO : DFRQ1_GLO) * fn);
                }
                /* pseudorange (m) */
                if (r[i] != 0.0 && pr[j] > -1E12)
                {
                    //obs->data[index].P[ind[k]] = r[i] + pr[j];
					obsd.P[ind[k]] = r[i] + pr[j];
                }
                /* carrier-phase (cycle) */
                if (r[i] != 0.0 && cp[j] > -1E12 && wl > 0.0)
                {
                    //obs->data[index].L[ind[k]] = (r[i] + cp[j]) / wl;
					obsd.L[ind[k]] = (r[i] + cp[j]) / wl;
                }
                /* doppler (hz) */
                if (rr && rrf && rrf[j] > -1E12 && wl > 0.0)
                {
                    //obs->data[index].D[ind[k]] = (float)(-(rr[i] + rrf[j]) / wl);
					obsd.D[ind[k]] = (float)(-(rr[i] + rrf[j]) / wl);
                }
                //obs->data[index].LLI[ind[k]] = (unsigned char)
                //                                   lossoflock(rtcm, sat, ind[k], lock[j]) +
                //                               (half[j] ? 3 : 0);
                //obs->data[index].SNR[ind[k]] = (unsigned char)(cnr[j] * 4.0);
                //obs->data[index].code[ind[k]] = code[k];

				obsd.LLI[ind[k]] = (unsigned char)lossoflock(rtcm, sat, ind[k], lock[j]) +
					               (half[j] ? 2 : 0);
				obsd.SNR[ind[k]] = (unsigned char)(cnr[j] * 4.0);
				obsd.code[ind[k]] = code[k];
				add_obs(&obsd, obs);
            }
            j++;
        }
    }
}
/* decode type msm message header --------------------------------------------*/
static int decode_msm_head(rtcm_t *rtcm, obs_t *obs, int sys, int *sync, int *iod,
                           msm_h_t *h, int *hsize)
{
    msm_h_t h0 = {0};
    double tow, tod;
    char tstr[64];
    int i = 24, j, dow, mask, staid, type, ncell = 0;

    type = rtcm_getbitu(rtcm->buff, i, 12);
    i += 12;

    *h = h0;
    if (i + 157 <= rtcm->len * 8)
    {
        staid = rtcm_getbitu(rtcm->buff, i, 12);
        i += 12;

        if (sys == _SYS_GLO_)
        {
            dow = rtcm_getbitu(rtcm->buff, i, 3);
            i += 3;
            tod = rtcm_getbitu(rtcm->buff, i, 27) * 0.001;
            i += 27;
            adjday_glot(&rtcm->time, tod);
        }
        else if (sys == _SYS_BDS_)
        {
            tow = rtcm_getbitu(rtcm->buff, i, 30) * 0.001;
            i += 30;
            tow += 14.0; /* BDT -> GPST */
            adjweek(&rtcm->time, tow);
        }
        else
        {
            tow = rtcm_getbitu(rtcm->buff, i, 30) * 0.001;
            i += 30;
            adjweek(&rtcm->time, tow);
        }
        *sync = rtcm_getbitu(rtcm->buff, i, 1);
        i += 1;
        *iod = rtcm_getbitu(rtcm->buff, i, 3);
        i += 3;
        h->time_s = (unsigned char)rtcm_getbitu(rtcm->buff, i, 7);
        i += 7;
        h->clk_str = (unsigned char)rtcm_getbitu(rtcm->buff, i, 2);
        i += 2;
        h->clk_ext = (unsigned char)rtcm_getbitu(rtcm->buff, i, 2);
        i += 2;
        h->smooth = (unsigned char)rtcm_getbitu(rtcm->buff, i, 1);
        i += 1;
        h->tint_s = (unsigned char)rtcm_getbitu(rtcm->buff, i, 3);
        i += 3;
        for (j = 1; j <= 64; j++)
        {
            mask = rtcm_getbitu(rtcm->buff, i, 1);
            i += 1;
            if (mask)
                h->sats[h->nsat++] = (unsigned char)j;
        }
        for (j = 1; j <= 32; j++)
        {
            mask = rtcm_getbitu(rtcm->buff, i, 1);
            i += 1;
            if (mask)
                h->sigs[h->nsig++] = (unsigned char)j;
        }
    }
    else
    {
        trace(2, "rtcm3 %d length error: len=%d\n", type, rtcm->len);
        return -1;
    }
    /* test station id */
    if (!test_staid(obs, staid))
        return -1;

    if (h->nsat * h->nsig > 64)
    {
        trace(2, "rtcm3 %d number of sats and sigs error: nsat=%d nsig=%d\n",
              type, h->nsat, h->nsig);
        return -1;
    }
    if (i + h->nsat * h->nsig > rtcm->len * 8)
    {
        trace(2, "rtcm3 %d length error: len=%d nsat=%d nsig=%d\n", type,
              rtcm->len, h->nsat, h->nsig);
        return -1;
    }
    for (j = 0; j < h->nsat * h->nsig; j++)
    {
        h->cellmask[j] = (unsigned char)rtcm_getbitu(rtcm->buff, i, 1);
        i += 1;
        if (h->cellmask[j])
            ncell++;
    }
    *hsize = i;

    time2str(rtcm->time, tstr, 2);
    trace(4, "decode_head_msm: time=%s sys=%c staid=%d nsat=%d nsig=%d sync=%d iod=%d ncell=%d\n",
          tstr, sys2char(sys), staid, h->nsat, h->nsig, *sync, *iod, ncell);

    return ncell;
}
/* decode unsupported msm message --------------------------------------------*/
static int decode_msm0(rtcm_t *rtcm, obs_t *obs, int sys)
{
    msm_h_t h = {0};
    int i, sync, iod;

    if (decode_msm_head(rtcm, obs, sys, &sync, &iod, &h, &i) < 0)
        return -1;
    obs->obsflag = !sync;

    return sync ? 0 : 1;
}
/* decode msm 4: full pseudorange and phaserange plus cnr --------------------*/
static int decode_msm4(rtcm_t *rtcm, obs_t *obs, int sys)
{
    msm_h_t h = {0};
    double r[64], pr[64], cp[64], cnr[64];
    int i, j, type, sync, iod, ncell, rng, rng_m, prv, cpv, lock[64], half[64];

    type = rtcm_getbitu(rtcm->buff, 24, 12);

    /* decode msm header */
    if ((ncell = decode_msm_head(rtcm, obs, sys, &sync, &iod, &h, &i)) < 0)
        return -1;

    if (i + h.nsat * 18 + ncell * 48 > rtcm->len * 8)
    {
        trace(2, "rtcm3 %d length error: nsat=%d ncell=%d len=%d\n", type, h.nsat,
              ncell, rtcm->len);
        return -1;
    }
    for (j = 0; j < h.nsat; j++)
        r[j] = 0.0;
    for (j = 0; j < ncell; j++)
        pr[j] = cp[j] = -1E16;

    /* decode satellite data */
    for (j = 0; j < h.nsat; j++)
    { /* range */
        rng = rtcm_getbitu(rtcm->buff, i, 8);
        i += 8;
        if (rng != 255)
            r[j] = rng * RANGE_MS;
    }
    for (j = 0; j < h.nsat; j++)
    {
        rng_m = rtcm_getbitu(rtcm->buff, i, 10);
        i += 10;
        if (r[j] != 0.0)
            r[j] += rng_m * P2_10 * RANGE_MS;
    }
    /* decode signal data */
    for (j = 0; j < ncell; j++)
    { /* pseudorange */
        prv = rtcm_getbits(rtcm->buff, i, 15);
        i += 15;
        if (prv != -16384)
            pr[j] = prv * P2_24 * RANGE_MS;
    }
    for (j = 0; j < ncell; j++)
    { /* phaserange */
        cpv = rtcm_getbits(rtcm->buff, i, 22);
        i += 22;
        if (cpv != -2097152)
            cp[j] = cpv * P2_29 * RANGE_MS;
    }
    for (j = 0; j < ncell; j++)
    { /* lock time */
        lock[j] = rtcm_getbitu(rtcm->buff, i, 4);
        i += 4;
    }
    for (j = 0; j < ncell; j++)
    { /* half-cycle ambiguity */
        half[j] = rtcm_getbitu(rtcm->buff, i, 1);
        i += 1;
    }
    for (j = 0; j < ncell; j++)
    { /* cnr */
        cnr[j] = rtcm_getbitu(rtcm->buff, i, 6) * 1.0;
        i += 6;
    }
    /* save obs data in msm message */
    save_msm_obs(rtcm, obs, sys, &h, r, pr, cp, NULL, NULL, cnr, lock, NULL, half);

    obs->obsflag = !sync;

    return sync ? 0 : 1;
}
/* decode msm 5: full pseudorange, phaserange, phaserangerate and cnr --------*/
static int decode_msm5(rtcm_t *rtcm, obs_t *obs, int sys)
{
    msm_h_t h = {0};
    double r[64], rr[64], pr[64], cp[64], rrf[64], cnr[64];
    int i, j, type, sync, iod, ncell, rng, rng_m, rate, prv, cpv, rrv, lock[64];
    int ex[64], half[64];

    type = rtcm_getbitu(rtcm->buff, 24, 12);

    /* decode msm header */
    if ((ncell = decode_msm_head(rtcm, obs, sys, &sync, &iod, &h, &i)) < 0)
        return -1;

    if (i + h.nsat * 36 + ncell * 63 > rtcm->len * 8)
    {
        trace(2, "rtcm3 %d length error: nsat=%d ncell=%d len=%d\n", type, h.nsat,
              ncell, rtcm->len);
        return -1;
    }
    for (j = 0; j < h.nsat; j++)
    {
        r[j] = rr[j] = 0.0;
        ex[j] = 15;
    }
    for (j = 0; j < ncell; j++)
        pr[j] = cp[j] = rrf[j] = -1E16;

    /* decode satellite data */
    for (j = 0; j < h.nsat; j++)
    { /* range */
        rng = rtcm_getbitu(rtcm->buff, i, 8);
        i += 8;
        if (rng != 255)
            r[j] = rng * RANGE_MS;
    }
    for (j = 0; j < h.nsat; j++)
    { /* extended info */
        ex[j] = rtcm_getbitu(rtcm->buff, i, 4);
        i += 4;
    }
    for (j = 0; j < h.nsat; j++)
    {
        rng_m = rtcm_getbitu(rtcm->buff, i, 10);
        i += 10;
        if (r[j] != 0.0)
            r[j] += rng_m * P2_10 * RANGE_MS;
    }
    for (j = 0; j < h.nsat; j++)
    { /* phaserangerate */
        rate = rtcm_getbits(rtcm->buff, i, 14);
        i += 14;
        if (rate != -8192)
            rr[j] = rate * 1.0;
    }
    /* decode signal data */
    for (j = 0; j < ncell; j++)
    { /* pseudorange */
        prv = rtcm_getbits(rtcm->buff, i, 15);
        i += 15;
        if (prv != -16384)
            pr[j] = prv * P2_24 * RANGE_MS;
    }
    for (j = 0; j < ncell; j++)
    { /* phaserange */
        cpv = rtcm_getbits(rtcm->buff, i, 22);
        i += 22;
        if (cpv != -2097152)
            cp[j] = cpv * P2_29 * RANGE_MS;
    }
    for (j = 0; j < ncell; j++)
    { /* lock time */
        lock[j] = rtcm_getbitu(rtcm->buff, i, 4);
        i += 4;
    }
    for (j = 0; j < ncell; j++)
    { /* half-cycle ambiguity */
        half[j] = rtcm_getbitu(rtcm->buff, i, 1);
        i += 1;
    }
    for (j = 0; j < ncell; j++)
    { /* cnr */
        cnr[j] = rtcm_getbitu(rtcm->buff, i, 6) * 1.0;
        i += 6;
    }
    for (j = 0; j < ncell; j++)
    { /* phaserangerate */
        rrv = rtcm_getbits(rtcm->buff, i, 15);
        i += 15;
        if (rrv != -16384)
            rrf[j] = rrv * 0.0001;
    }
    /* save obs data in msm message */
    save_msm_obs(rtcm, obs, sys, &h, r, pr, cp, rr, rrf, cnr, lock, ex, half);

    obs->obsflag = !sync;

    return sync ? 0 : 1;
}
/* decode msm 6: full pseudorange and phaserange plus cnr (high-res) ---------*/
static int decode_msm6(rtcm_t *rtcm, obs_t *obs, int sys)
{
    msm_h_t h = {0};
    double r[64], pr[64], cp[64], cnr[64];
    int i, j, type, sync, iod, ncell, rng, rng_m, prv, cpv, lock[64], half[64];

    type = rtcm_getbitu(rtcm->buff, 24, 12);

    /* decode msm header */
    if ((ncell = decode_msm_head(rtcm, obs, sys, &sync, &iod, &h, &i)) < 0)
        return -1;

    if (i + h.nsat * 18 + ncell * 65 > rtcm->len * 8)
    {
        trace(2, "rtcm3 %d length error: nsat=%d ncell=%d len=%d\n", type, h.nsat,
              ncell, rtcm->len);
        return -1;
    }
    for (j = 0; j < h.nsat; j++)
        r[j] = 0.0;
    for (j = 0; j < ncell; j++)
        pr[j] = cp[j] = -1E16;

    /* decode satellite data */
    for (j = 0; j < h.nsat; j++)
    { /* range */
        rng = rtcm_getbitu(rtcm->buff, i, 8);
        i += 8;
        if (rng != 255)
            r[j] = rng * RANGE_MS;
    }
    for (j = 0; j < h.nsat; j++)
    {
        rng_m = rtcm_getbitu(rtcm->buff, i, 10);
        i += 10;
        if (r[j] != 0.0)
            r[j] += rng_m * P2_10 * RANGE_MS;
    }
    /* decode signal data */
    for (j = 0; j < ncell; j++)
    { /* pseudorange */
        prv = rtcm_getbits(rtcm->buff, i, 20);
        i += 20;
        if (prv != -524288)
            pr[j] = prv * P2_29 * RANGE_MS;
    }
    for (j = 0; j < ncell; j++)
    { /* phaserange */
        cpv = rtcm_getbits(rtcm->buff, i, 24);
        i += 24;
        if (cpv != -8388608)
            cp[j] = cpv * P2_31 * RANGE_MS;
    }
    for (j = 0; j < ncell; j++)
    { /* lock time */
        lock[j] = rtcm_getbitu(rtcm->buff, i, 10);
        i += 10;
    }
    for (j = 0; j < ncell; j++)
    { /* half-cycle ambiguity */
        half[j] = rtcm_getbitu(rtcm->buff, i, 1);
        i += 1;
    }
    for (j = 0; j < ncell; j++)
    { /* cnr */
        cnr[j] = rtcm_getbitu(rtcm->buff, i, 10) * 0.0625;
        i += 10;
    }
    /* save obs data in msm message */
    save_msm_obs(rtcm, obs, sys, &h, r, pr, cp, NULL, NULL, cnr, lock, NULL, half);

    obs->obsflag = !sync;

    return sync ? 0 : 1;
}
/* decode msm 7: full pseudorange, phaserange, phaserangerate and cnr (h-res) */
static int decode_msm7(rtcm_t *rtcm, obs_t *obs, int sys)
{
    msm_h_t h = {0};
    double r[64] = {0}, rr[64] = {0}, pr[64] = {0}, cp[64] = {0}, rrf[64] = {0}, cnr[64] = {0};
    int i, j, type, sync, iod, ncell, rng, rng_m, rate, prv, cpv, rrv, lock[64];
    int ex[64] = {0}, half[64] = {0};

    type = rtcm_getbitu(rtcm->buff, 24, 12);

    /* decode msm header */
    if ((ncell = decode_msm_head(rtcm, obs, sys, &sync, &iod, &h, &i)) < 0)
        return -1;

    if (i + h.nsat * 36 + ncell * 80 > rtcm->len * 8)
    {
        trace(2, "rtcm3 %d length error: nsat=%d ncell=%d len=%d\n", type, h.nsat,
              ncell, rtcm->len);
        return -1;
    }
    for (j = 0; j < h.nsat; j++)
    {
        r[j] = rr[j] = 0.0;
        ex[j] = 15;
    }
    for (j = 0; j < ncell; j++)
        pr[j] = cp[j] = rrf[j] = -1E16;

    /* decode satellite data */
    for (j = 0; j < h.nsat; j++)
    { /* range */
        rng = rtcm_getbitu(rtcm->buff, i, 8);
        i += 8;
        if (rng != 255)
            r[j] = rng * RANGE_MS;
    }
    for (j = 0; j < h.nsat; j++)
    { /* extended info */
        ex[j] = rtcm_getbitu(rtcm->buff, i, 4);
        i += 4;
    }
    for (j = 0; j < h.nsat; j++)
    {
        rng_m = rtcm_getbitu(rtcm->buff, i, 10);
        i += 10;
        if (r[j] != 0.0)
            r[j] += rng_m * P2_10 * RANGE_MS;
    }
    for (j = 0; j < h.nsat; j++)
    { /* phaserangerate */
        rate = rtcm_getbits(rtcm->buff, i, 14);
        i += 14;
        if (rate != -8192)
            rr[j] = rate * 1.0;
    }
    /* decode signal data */
    for (j = 0; j < ncell; j++)
    { /* pseudorange */
        prv = rtcm_getbits(rtcm->buff, i, 20);
        i += 20;
        if (prv != -524288)
            pr[j] = prv * P2_29 * RANGE_MS;
    }
    for (j = 0; j < ncell; j++)
    { /* phaserange */
        cpv = rtcm_getbits(rtcm->buff, i, 24);
        i += 24;
        if (cpv != -8388608)
            cp[j] = cpv * P2_31 * RANGE_MS;
    }
    for (j = 0; j < ncell; j++)
    { /* lock time */
        lock[j] = rtcm_getbitu(rtcm->buff, i, 10);
        i += 10;
    }
    for (j = 0; j < ncell; j++)
    { /* half-cycle amiguity */
        half[j] = rtcm_getbitu(rtcm->buff, i, 1);
        i += 1;
    }
    for (j = 0; j < ncell; j++)
    { /* cnr */
        cnr[j] = rtcm_getbitu(rtcm->buff, i, 10) * 0.0625;
        i += 10;
    }
    for (j = 0; j < ncell; j++)
    { /* phaserangerate */
        rrv = rtcm_getbits(rtcm->buff, i, 15);
        i += 15;
        if (rrv != -16384)
            rrf[j] = rrv * 0.0001;
    }
    /* save obs data in msm message */
    save_msm_obs(rtcm, obs, sys, &h, r, pr, cp, rr, rrf, cnr, lock, ex, half);

    obs->obsflag = !sync;

    return sync ? 0 : 1;
}

/* decode type 1230: glonass L1 and L2 code-phase biases ---------------------*/
static int decode_type1230(rtcm_t *rtcm)
{
    trace(2, "rtcm3 1230: not supported message\n");
    return 0;
}
#ifdef _USE_PPP_
/* decode ssr 1,4 message header ---------------------------------------------*/
static int decode_ssr1_head(rtcm_t *rtcm, int sys, int *sync, int *iod,
                            double *udint, int *refd, int *hsize)
{
    double tod, tow;
    char tstr[64];
    int i = 24 + 12, nsat, udi, provid = 0, solid = 0, ns;

    ns = sys == _SYS_QZS_ ? 4 : 6;

    if (i + (sys == _SYS_GLO_ ? 53 : 50 + ns) > rtcm->len * 8)
        return -1;

    if (sys == _SYS_GLO_)
    {
        tod = rtcm_getbitu(rtcm->buff, i, 17);
        i += 17;
        adjday_glot(&rtcm->time, tod);
    }
    else
    {
        tow = rtcm_getbitu(rtcm->buff, i, 20);
        i += 20;
        adjweek(&rtcm->time, tow);
    }
    udi = rtcm_getbitu(rtcm->buff, i, 4);
    i += 4;
    *sync = rtcm_getbitu(rtcm->buff, i, 1);
    i += 1;
    *refd = rtcm_getbitu(rtcm->buff, i, 1);
    i += 1; /* satellite ref datum */
    *iod = rtcm_getbitu(rtcm->buff, i, 4);
    i += 4; /* iod */
    provid = rtcm_getbitu(rtcm->buff, i, 16);
    i += 16; /* provider id */
    solid = rtcm_getbitu(rtcm->buff, i, 4);
    i += 4; /* solution id */
    nsat = rtcm_getbitu(rtcm->buff, i, ns);
    i += ns;
    *udint = ssrudint[udi];

    time2str(rtcm->time, tstr, 2);
    trace(4, "decode_ssr1_head: time=%s sys=%c nsat=%d sync=%d iod=%d provid=%d solid=%d\n",
          tstr, sys2char(sys), nsat, *sync, *iod, provid, solid);

    *hsize = i;
    return nsat;
}
/* decode ssr 2,3,5,6 message header -----------------------------------------*/
static int decode_ssr2_head(rtcm_t *rtcm, int sys, int *sync, int *iod,
                            double *udint, int *hsize)
{
    double tod, tow;
    char tstr[64];
    int i = 24 + 12, nsat, udi, provid = 0, solid = 0, ns;

    ns = sys == _SYS_QZS_ ? 4 : 6;

    if (i + (sys == _SYS_GLO_ ? 52 : 49 + ns) > rtcm->len * 8)
        return -1;

    if (sys == _SYS_GLO_)
    {
        tod = rtcm_getbitu(rtcm->buff, i, 17);
        i += 17;
        adjday_glot(&rtcm->time, tod);
    }
    else
    {
        tow = rtcm_getbitu(rtcm->buff, i, 20);
        i += 20;
        adjweek(&rtcm->time, tow);
    }
    udi = rtcm_getbitu(rtcm->buff, i, 4);
    i += 4;
    *sync = rtcm_getbitu(rtcm->buff, i, 1);
    i += 1;
    *iod = rtcm_getbitu(rtcm->buff, i, 4);
    i += 4;
    provid = rtcm_getbitu(rtcm->buff, i, 16);
    i += 16; /* provider id */
    solid = rtcm_getbitu(rtcm->buff, i, 4);
    i += 4; /* solution id */
    nsat = rtcm_getbitu(rtcm->buff, i, ns);
    i += ns;
    *udint = ssrudint[udi];

    time2str(rtcm->time, tstr, 2);
    trace(4, "decode_ssr2_head: time=%s sys=%d nsat=%c sync=%d iod=%d provid=%d solid=%d\n",
          tstr, sys2char(sys), nsat, *sync, *iod, provid, solid);

    *hsize = i;
    return nsat;
}

/* decode ssr 1: orbit corrections -------------------------------------------*/
static int decode_ssr1(rtcm_t *rtcm, int sys, nav_t *nav, obs_t *obs)
{
    double udint, deph[3], ddeph[3];
    int i, j, k, type, sync, iod, nsat, prn, sat, iode, iodcrc, refd = 0, np, ni, nj, offp, loc;
    ssr_t ssr = {0};

    type = rtcm_getbitu(rtcm->buff, 24, 12);

    if ((nsat = decode_ssr1_head(rtcm, sys, &sync, &iod, &udint, &refd, &i)) < 0)
    {
        trace(2, "rtcm3 %d length error: len=%d\n", type, rtcm->len);
        return -1;
    }
    switch (sys)
    {
    case _SYS_GPS_:
        np = 6;
        ni = 8;
        nj = 0;
        offp = 0;
        break;
    case _SYS_GLO_:
        np = 5;
        ni = 8;
        nj = 0;
        offp = 0;
        break;
    case _SYS_GAL_:
        np = 6;
        ni = 10;
        nj = 0;
        offp = 0;
        break;
    case _SYS_QZS_:
        np = 4;
        ni = 8;
        nj = 0;
        offp = 192;
        break;
    case _SYS_BDS_:
        np = 6;
        ni = 10;
        nj = 24;
        offp = 1;
        break;
    case _SYS_SBS_:
        np = 6;
        ni = 9;
        nj = 24;
        offp = 120;
        break;
    default:
        return sync ? 0 : 10;
    }
    for (j = 0; j < nsat && i + 121 + np + ni + nj <= rtcm->len * 8; j++)
    {
        prn = rtcm_getbitu(rtcm->buff, i, np) + offp;
        i += np;
        iode = rtcm_getbitu(rtcm->buff, i, ni);
        i += ni;
        iodcrc = rtcm_getbitu(rtcm->buff, i, nj);
        i += nj;
        deph[0] = rtcm_getbits(rtcm->buff, i, 22) * 1E-4;
        i += 22;
        deph[1] = rtcm_getbits(rtcm->buff, i, 20) * 4E-4;
        i += 20;
        deph[2] = rtcm_getbits(rtcm->buff, i, 20) * 4E-4;
        i += 20;
        ddeph[0] = rtcm_getbits(rtcm->buff, i, 21) * 1E-6;
        i += 21;
        ddeph[1] = rtcm_getbits(rtcm->buff, i, 19) * 4E-6;
        i += 19;
        ddeph[2] = rtcm_getbits(rtcm->buff, i, 19) * 4E-6;
        i += 19;

        if (!(sat = satno(sys, prn)))
        {
            /*         trace(2,"rtcm3 %d satellite number error: prn=%d\n",type,prn);   */
            continue;
        }
        memset(&ssr, 0, sizeof(ssr_t));
        ssr.sat = sat;
        ssr.t0[0] = rtcm->time;
        ssr.udi[0] = udint;
        ssr.iod[0] = iod;
        ssr.iode = iode;     /* sbas/bds: toe/t0 modulo */
        ssr.iodcrc = iodcrc; /* sbas/bds: iod crc */
        ssr.refd = refd;

        for (k = 0; k < 3; k++)
        {
            ssr.deph[k] = deph[k];
            ssr.ddeph[k] = ddeph[k];
        }
        ssr.update = 1;

        for (loc = 0; loc < nav->ns; ++loc)
        {
            if (nav->ssr[loc].sat == sat)
            {
                break;
            }
        }
        if (loc < nav->ns)
        {
            nav->ssr[loc] = ssr;
        }
        else if (loc == nav->ns && loc < MAXSSR)
        {
            nav->ssr[nav->ns] = ssr;
            ++nav->ns;
        }
    }

    return sync ? 0 : 10;
}
/* decode ssr 2: clock corrections -------------------------------------------*/
static int decode_ssr2(rtcm_t *rtcm, int sys, nav_t *nav, obs_t *obs)
{
    double udint, dclk[3];
    int i, j, k, type, sync, iod, nsat, prn, sat, np, offp, loc;
    ssr_t ssr = {0};

    type = rtcm_getbitu(rtcm->buff, 24, 12);

    if ((nsat = decode_ssr2_head(rtcm, sys, &sync, &iod, &udint, &i)) < 0)
    {
        trace(2, "rtcm3 %d length error: len=%d\n", type, rtcm->len);
        return -1;
    }
    switch (sys)
    {
    case _SYS_GPS_:
        np = 6;
        offp = 0;
        break;
    case _SYS_GLO_:
        np = 5;
        offp = 0;
        break;
    case _SYS_GAL_:
        np = 6;
        offp = 0;
        break;
    case _SYS_QZS_:
        np = 4;
        offp = 192;
        break;
    case _SYS_BDS_:
        np = 6;
        offp = 1;
        break;
    case _SYS_SBS_:
        np = 6;
        offp = 120;
        break;
    default:
        return sync ? 0 : 10;
    }
    for (j = 0; j < nsat && i + 70 + np <= rtcm->len * 8; j++)
    {
        prn = rtcm_getbitu(rtcm->buff, i, np) + offp;
        i += np;
        dclk[0] = rtcm_getbits(rtcm->buff, i, 22) * 1E-4;
        i += 22;
        dclk[1] = rtcm_getbits(rtcm->buff, i, 21) * 1E-6;
        i += 21;
        dclk[2] = rtcm_getbits(rtcm->buff, i, 27) * 2E-8;
        i += 27;

        if (!(sat = satno(sys, prn)))
        {
            /*         trace(2,"rtcm3 %d satellite number error: prn=%d\n",type,prn);    */
            continue;
        }
        memset(&ssr, 0, sizeof(ssr_t));
        ssr.sat = sat;
        ssr.t0[1] = rtcm->time;
        ssr.udi[1] = udint;
        ssr.iod[1] = iod;

        for (k = 0; k < 3; k++)
        {
            ssr.dclk[k] = dclk[k];
        }
        ssr.update = 1;

        for (loc = 0; loc < nav->ns; ++loc)
        {
            if (nav->ssr[loc].sat == sat)
            {
                break;
            }
        }
        if (loc < nav->ns)
        {
            nav->ssr[loc] = ssr;
        }
        else if (loc == nav->ns && loc < MAXSSR)
        {
            nav->ssr[nav->ns] = ssr;
            ++nav->ns;
        }
    }

    return sync ? 0 : 10;
}
/* decode ssr 3: satellite code biases ---------------------------------------*/
static int decode_ssr3(rtcm_t *rtcm, int sys, nav_t *nav, obs_t *obs)
{
    const int *codes;
    const unsigned char *freqs;
    double udint, bias; // cbias[NFREQ];
    int i, j, k, type, mode, sync, iod, nsat, prn, sat, nbias, np, offp, ncode, loc;
    ssr_t ssr = {0};
    memset(&ssr, 0, sizeof(ssr_t));
    type = rtcm_getbitu(rtcm->buff, 24, 12);

    if ((nsat = decode_ssr2_head(rtcm, sys, &sync, &iod, &udint, &i)) < 0)
    {
        trace(2, "rtcm3 %d length error: len=%d\n", type, rtcm->len);
        return -1;
    }
    switch (sys)
    {
    case _SYS_GPS_:
        np = 6;
        offp = 0;
        codes = codes_gps;
        ncode = 17;
        freqs = obsfreqs_gps;
        break;
    case _SYS_GLO_:
        np = 5;
        offp = 0;
        codes = codes_glo;
        ncode = 4;
        freqs = obsfreqs_glo;
        break;
    case _SYS_GAL_:
        np = 6;
        offp = 0;
        codes = codes_gal;
        ncode = 19;
        freqs = obsfreqs_gal;
        break;
    case _SYS_QZS_:
        np = 4;
        offp = 192;
        codes = codes_qzs;
        ncode = 13;
        freqs = obsfreqs_qzs;
        break;
    case _SYS_BDS_:
        np = 6;
        offp = 1;
        codes = codes_bds;
        ncode = 9;
        freqs = obsfreqs_cmp;
        break;
    case _SYS_SBS_:
        np = 6;
        offp = 120;
        codes = codes_sbs;
        ncode = 4;
        freqs = obsfreqs_sbs;
        break;
    default:
        return sync ? 0 : 10;
    }
    for (j = 0; j < nsat && i + 5 + np <= rtcm->len * 8; j++)
    {
        prn = rtcm_getbitu(rtcm->buff, i, np) + offp;
        i += np;
        nbias = rtcm_getbitu(rtcm->buff, i, 5);
        i += 5;

        for (k = 0; k < nbias && i + 19 <= rtcm->len * 8; k++)
        {
            mode = rtcm_getbitu(rtcm->buff, i, 5);
            i += 5;
            bias = rtcm_getbits(rtcm->buff, i, 14) * 0.01;
            i += 14;
            if (mode <= ncode && bias != 0.0 && freqs[codes[mode] + 1] <= NFREQ)
            {
                //  ssr.cbias[freqs[codes[mode] + 1] - 1] = (float)bias;
            }
        }
        if (!(sat = satno(sys, prn)))
        {
            /*      trace(2,"rtcm3 %d satellite number error: prn=%d\n",type,prn);   */
            continue;
        }

        ssr.sat = sat;
        ssr.t0[4] = rtcm->time;
        ssr.udi[4] = udint;
        ssr.iod[4] = iod;
        ssr.update = 1;

        for (loc = 0; loc < nav->ns; ++loc)
        {
            if (nav->ssr[loc].sat == sat)
            {
                nav->ssr[loc].t0[4] = ssr.t0[4];
                nav->ssr[loc].udi[4] = ssr.udi[4];
                nav->ssr[loc].iod[4] = ssr.iod[4];
                //   for (k = 0; k < NFREQ; k++)   nav->ssr[loc].cbias[k] = ssr.cbias[k];
                break;
            }
        }

        if (loc < nav->ns)
        {
            for (k = 0; k < obs->n; k++)
            {
                if (obs->data[k].sat == sat)
                {
                    nav->ssr[loc].t0[4] = ssr.t0[4];
                    nav->ssr[loc].udi[4] = ssr.udi[4];
                    nav->ssr[loc].iod[4] = ssr.iod[4];
                    //       for (k = 0; k < NFREQ; k++)   nav->ssr[loc].cbias[k] = ssr.cbias[k];
                    break;
                }
            }
        }
        if (loc == nav->ns)
        {
            if (loc < MAXSSR)
            {
                for (k = 0; k < obs->n; k++)
                {
                    if (obs->data[k].sat == sat)
                    {
                        nav->ssr[nav->ns] = ssr;
                        ++nav->ns;
                        break;
                    }
                }
            }
            else
            {
                for (j = 0; j < nav->ns; ++j)
                {
                    int idx = -1;
                    for (k = 0; k < obs->n; k++)
                    {
                        if (obs->data[k].sat == nav->ssr[j].sat)
                        {
                            idx = k;
                            break;
                        }
                    }
                    if (idx == -1)
                    {
                        for (k = 0; k < obs->n; k++)
                        {
                            if (obs->data[k].sat == sat)
                            {
                                nav->ssr[j] = ssr;
                                return sync ? 0 : 10;
                            }
                        }
                    }
                }
            }
        }
    }
    return sync ? 0 : 10;
}
/* decode ssr 4: combined orbit and clock corrections ------------------------*/
static int decode_ssr4(rtcm_t *rtcm, int sys, nav_t *nav, obs_t *obs)
{
    double udint, deph[3], ddeph[3], dclk[3];
    int i, j, k, type, nsat, sync, iod, prn, sat, iode, iodcrc, refd = 0, np, ni, nj, offp, loc;
    ssr_t ssr = {0};

    type = rtcm_getbitu(rtcm->buff, 24, 12);

    if ((nsat = decode_ssr1_head(rtcm, sys, &sync, &iod, &udint, &refd, &i)) < 0)
    {
        trace(2, "rtcm3 %d length error: len=%d\n", type, rtcm->len);
        return -1;
    }
    switch (sys)
    {
    case _SYS_GPS_:
        np = 6;
        ni = 8;
        nj = 0;
        offp = 0;
        break;
    case _SYS_GLO_:
        np = 5;
        ni = 8;
        nj = 0;
        offp = 0;
        break;
    case _SYS_GAL_:
        np = 6;
        ni = 10;
        nj = 0;
        offp = 0;
        break;
    case _SYS_QZS_:
        np = 4;
        ni = 8;
        nj = 0;
        offp = 192;
        break;
    case _SYS_BDS_:
        np = 6;
        ni = 10;
        nj = 24;
        offp = 1;
        break;
    case _SYS_SBS_:
        np = 6;
        ni = 9;
        nj = 24;
        offp = 120;
        break;
    default:
        return sync ? 0 : 10;
    }
    for (j = 0; j < nsat && i + 191 + np + ni + nj <= rtcm->len * 8; j++)
    {
        prn = rtcm_getbitu(rtcm->buff, i, np) + offp;
        i += np;
        iode = rtcm_getbitu(rtcm->buff, i, ni);
        i += ni;
        iodcrc = rtcm_getbitu(rtcm->buff, i, nj);
        i += nj;
        deph[0] = rtcm_getbits(rtcm->buff, i, 22) * 1E-4;
        i += 22;
        deph[1] = rtcm_getbits(rtcm->buff, i, 20) * 4E-4;
        i += 20;
        deph[2] = rtcm_getbits(rtcm->buff, i, 20) * 4E-4;
        i += 20;
        ddeph[0] = rtcm_getbits(rtcm->buff, i, 21) * 1E-6;
        i += 21;
        ddeph[1] = rtcm_getbits(rtcm->buff, i, 19) * 4E-6;
        i += 19;
        ddeph[2] = rtcm_getbits(rtcm->buff, i, 19) * 4E-6;
        i += 19;

        dclk[0] = rtcm_getbits(rtcm->buff, i, 22) * 1E-4;
        i += 22;
        dclk[1] = rtcm_getbits(rtcm->buff, i, 21) * 1E-6;
        i += 21;
        dclk[2] = rtcm_getbits(rtcm->buff, i, 27) * 2E-8;
        i += 27;

        if (!(sat = satno(sys, prn)))
        {
            /*        trace(2,"rtcm3 %d satellite number error: prn=%d\n",type,prn);  */
            continue;
        }
        memset(&ssr, 0, sizeof(ssr_t));
        ssr.sat = sat;
        ssr.t0[0] = ssr.t0[1] = rtcm->time;
        ssr.udi[0] = ssr.udi[1] = udint;
        ssr.iod[0] = ssr.iod[1] = iod;
        ssr.iode = iode;
        ssr.iodcrc = iodcrc;
        ssr.refd = refd;

        for (k = 0; k < 3; k++)
        {
            ssr.deph[k] = deph[k];
            ssr.ddeph[k] = ddeph[k];
            ssr.dclk[k] = dclk[k];
        }
        ssr.update = 1;

        for (loc = 0; loc < nav->ns; ++loc)
        {
            if (nav->ssr[loc].sat == sat)
            {
                break;
            }
        }

        if (loc < nav->ns)
        {
            for (k = 0; k < obs->n; k++)
            {
                if (obs->data[k].sat == sat)
                {
                    nav->ssr[loc] = ssr;
                    break;
                }
            }
        }
        else if (loc == nav->ns)
        {
            if (loc < MAXSSR)
            {
                for (k = 0; k < obs->n; k++)
                {
                    if (obs->data[k].sat == sat)
                    {
                        nav->ssr[nav->ns] = ssr;
                        ++nav->ns;
                        break;
                    }
                }
            }
            else
            {
                for (j = 0; j < nav->ns; ++j)
                {
                    int idx = -1;
                    for (k = 0; k < obs->n; k++)
                    {
                        if (obs->data[k].sat == nav->ssr[j].sat)
                        {
                            idx = k;
                            break;
                        }
                    }
                    if (idx == -1)
                    {
                        for (k = 0; k < obs->n; k++)
                        {
                            if (obs->data[k].sat == sat)
                            {
                                nav->ssr[j] = ssr;
                                return sync ? 0 : 10;
                            }
                        }
                    }
                }
            }
        }
    }
    return sync ? 0 : 10;
}
/* decode ssr 5: ura ---------------------------------------------------------*/
static int decode_ssr5(rtcm_t *rtcm, int sys, nav_t *nav)
{
    double udint;
    int i, j, type, nsat, sync, iod, prn, sat, ura, np, offp, loc;
    ssr_t ssr = {0};
    ;

    type = rtcm_getbitu(rtcm->buff, 24, 12);

    if ((nsat = decode_ssr2_head(rtcm, sys, &sync, &iod, &udint, &i)) < 0)
    {
        trace(2, "rtcm3 %d length error: len=%d\n", type, rtcm->len);
        return -1;
    }
    switch (sys)
    {
    case _SYS_GPS_:
        np = 6;
        offp = 0;
        break;
    case _SYS_GLO_:
        np = 5;
        offp = 0;
        break;
    case _SYS_GAL_:
        np = 6;
        offp = 0;
        break;
    case _SYS_QZS_:
        np = 4;
        offp = 192;
        break;
    case _SYS_BDS_:
        np = 6;
        offp = 1;
        break;
    case _SYS_SBS_:
        np = 6;
        offp = 120;
        break;
    default:
        return sync ? 0 : 10;
    }
    for (j = 0; j < nsat && i + 6 + np <= rtcm->len * 8; j++)
    {
        prn = rtcm_getbitu(rtcm->buff, i, np) + offp;
        i += np;
        ura = rtcm_getbitu(rtcm->buff, i, 6);
        i += 6;

        if (!(sat = satno(sys, prn)))
        {
            trace(2, "rtcm3 %d satellite number error: prn=%d\n", type, prn);
            continue;
        }
        memset(&ssr, 0, sizeof(ssr_t));
        ssr.sat = sat;
        ssr.t0[3] = rtcm->time;
        ssr.udi[3] = udint;
        ssr.iod[3] = iod;
        ssr.ura = ura;
        ssr.update = 1;

        for (loc = 0; loc < nav->ns; ++loc)
        {
            if (nav->ssr[loc].sat == sat)
            {
                break;
            }
        }
        if (loc < nav->ns)
        {
            nav->ssr[loc] = ssr;
        }
        else if (loc == nav->ns && loc < MAXEPH)
        {
            nav->ssr[nav->ns] = ssr;
            ++nav->ns;
        }
    }
    return sync ? 0 : 10;
}
/* decode ssr 6: high rate clock correction ----------------------------------*/
static int decode_ssr6(rtcm_t *rtcm, int sys, nav_t *nav)
{
    double udint, hrclk;
    int i, j, type, nsat, sync, iod, prn, sat, np, offp, loc;
    ssr_t ssr = {0};

    type = rtcm_getbitu(rtcm->buff, 24, 12);

    if ((nsat = decode_ssr2_head(rtcm, sys, &sync, &iod, &udint, &i)) < 0)
    {
        trace(2, "rtcm3 %d length error: len=%d\n", type, rtcm->len);
        return -1;
    }
    switch (sys)
    {
    case _SYS_GPS_:
        np = 6;
        offp = 0;
        break;
    case _SYS_GLO_:
        np = 5;
        offp = 0;
        break;
    case _SYS_GAL_:
        np = 6;
        offp = 0;
        break;
    case _SYS_QZS_:
        np = 4;
        offp = 192;
        break;
    case _SYS_BDS_:
        np = 6;
        offp = 1;
        break;
    case _SYS_SBS_:
        np = 6;
        offp = 120;
        break;
    default:
        return sync ? 0 : 10;
    }
    for (j = 0; j < nsat && i + 22 + np <= rtcm->len * 8; j++)
    {
        prn = rtcm_getbitu(rtcm->buff, i, np) + offp;
        i += np;
        hrclk = rtcm_getbits(rtcm->buff, i, 22) * 1E-4;
        i += 22;

        if (!(sat = satno(sys, prn)))
        {
            trace(2, "rtcm3 %d satellite number error: prn=%d\n", type, prn);
            continue;
        }
        memset(&ssr, 0, sizeof(ssr_t));
        ssr.sat = sat;
        ssr.t0[2] = rtcm->time;
        ssr.udi[2] = udint;
        ssr.iod[2] = iod;
        ssr.hrclk = hrclk;
        ssr.update = 1;

        for (loc = 0; loc < nav->ns; ++loc)
        {
            if (nav->ssr[loc].sat == sat)
            {
                break;
            }
        }
        if (loc < nav->ns)
        {
            nav->ssr[loc] = ssr;
        }
        else if (loc == nav->ns && loc < MAXEPH)
        {
            nav->ssr[nav->ns] = ssr;
            ++nav->ns;
        }
    }
    return sync ? 0 : 10;
}
/* decode ssr 7 message header -----------------------------------------------*/
static int decode_ssr7_head(rtcm_t *rtcm, int sys, int *sync, int *iod,
                            double *udint, int *dispe, int *mw, int *hsize)
{
    double tod, tow;
    char tstr[64];
    int i = 24 + 12, nsat, udi, provid = 0, solid = 0, ns;

    ns = sys == _SYS_QZS_ ? 4 : 6;

    if (i + (sys == _SYS_GLO_ ? 54 : 51 + ns) > rtcm->len * 8)
        return -1;

    if (sys == _SYS_GLO_)
    {
        tod = rtcm_getbitu(rtcm->buff, i, 17);
        i += 17;
        adjday_glot(&rtcm->time, tod);
    }
    else
    {
        tow = rtcm_getbitu(rtcm->buff, i, 20);
        i += 20;
        adjweek(&rtcm->time, tow);
    }
    udi = rtcm_getbitu(rtcm->buff, i, 4);
    i += 4;
    *sync = rtcm_getbitu(rtcm->buff, i, 1);
    i += 1;
    *iod = rtcm_getbitu(rtcm->buff, i, 4);
    i += 4;
    provid = rtcm_getbitu(rtcm->buff, i, 16);
    i += 16; /* provider id */
    solid = rtcm_getbitu(rtcm->buff, i, 4);
    i += 4; /* solution id */
    *dispe = rtcm_getbitu(rtcm->buff, i, 1);
    i += 1; /* dispersive bias consistency ind */
    *mw = rtcm_getbitu(rtcm->buff, i, 1);
    i += 1; /* MW consistency indicator */
    nsat = rtcm_getbitu(rtcm->buff, i, ns);
    i += ns;
    *udint = ssrudint[udi];

    time2str(rtcm->time, tstr, 2);
    trace(4, "decode_ssr7_head: time=%s sys=%c nsat=%d sync=%d iod=%d provid=%d solid=%d\n",
          tstr, sys2char(sys), nsat, *sync, *iod, provid, solid);

    *hsize = i;
    return nsat;
}
/* decode ssr 7: phase bias --------------------------------------------------*/
static int decode_ssr7(rtcm_t *rtcm, int sys, nav_t *nav)
{
    const int *codes;
    double udint, bias, std, pbias[MAXCODE], stdpb[MAXCODE];
    int i, j, k, type, mode, sync, iod, nsat, prn, sat, nbias, ncode, np, mw, offp, sii, swl;
    int dispe, sdc, yaw_ang, yaw_rate;
    int loc;
    ssr_t ssr = {0};

    type = rtcm_getbitu(rtcm->buff, 24, 12);

    if ((nsat = decode_ssr7_head(rtcm, sys, &sync, &iod, &udint, &dispe, &mw, &i)) < 0)
    {
        trace(2, "rtcm3 %d length error: len=%d\n", type, rtcm->len);
        return -1;
    }
    switch (sys)
    {
    case _SYS_GPS_:
        np = 6;
        offp = 0;
        codes = codes_gps;
        ncode = 17;
        break;
    case _SYS_GLO_:
        np = 5;
        offp = 0;
        codes = codes_glo;
        ncode = 4;
        break;
    case _SYS_GAL_:
        np = 6;
        offp = 0;
        codes = codes_gal;
        ncode = 19;
        break;
    case _SYS_QZS_:
        np = 4;
        offp = 192;
        codes = codes_qzs;
        ncode = 13;
        break;
    case _SYS_BDS_:
        np = 6;
        offp = 1;
        codes = codes_bds;
        ncode = 9;
        break;
    default:
        return sync ? 0 : 10;
    }
    for (j = 0; j < nsat && i + 5 + 17 + np <= rtcm->len * 8; j++)
    {
        prn = rtcm_getbitu(rtcm->buff, i, np) + offp;
        i += np;
        nbias = rtcm_getbitu(rtcm->buff, i, 5);
        i += 5;
        yaw_ang = rtcm_getbitu(rtcm->buff, i, 9);
        i += 9;
        yaw_rate = rtcm_getbits(rtcm->buff, i, 8);
        i += 8;

        for (k = 0; k < MAXCODE; k++)
            pbias[k] = stdpb[k] = 0.0;
        for (k = 0; k < nbias && i + 49 <= rtcm->len * 8; k++)
        {
            mode = rtcm_getbitu(rtcm->buff, i, 5);
            i += 5;
            sii = rtcm_getbitu(rtcm->buff, i, 1);
            i += 1; /* integer-indicator */
            swl = rtcm_getbitu(rtcm->buff, i, 2);
            i += 2; /* WL integer-indicator */
            sdc = rtcm_getbitu(rtcm->buff, i, 4);
            i += 4; /* discontinuity counter */
            bias = rtcm_getbits(rtcm->buff, i, 20);
            i += 20; /* phase bias (m) */
            std = rtcm_getbitu(rtcm->buff, i, 17);
            i += 17; /* phase bias std-dev (m) */
            if (mode <= ncode)
            {
                pbias[codes[mode] - 1] = bias * 0.0001; /* (m) */
                stdpb[codes[mode] - 1] = std * 0.0001;  /* (m) */
            }
            else
            {
                trace(2, "rtcm3 %d not supported mode: mode=%d\n", type, mode);
            }
        }
        if (!(sat = satno(sys, prn)))
        {
            trace(2, "rtcm3 %d satellite number error: prn=%c%02d\n", type, sys2char(sys), prn);
            continue;
        }
        memset(&ssr, 0, sizeof(ssr_t));
        ssr.sat = sat;
        ssr.t0[5] = rtcm->time;
        ssr.udi[5] = udint;
        ssr.iod[5] = iod;
        ssr.yaw_ang = yaw_ang / 256.0 * 180.0;    /* (deg) */
        ssr.yaw_rate = yaw_rate / 8192.0 * 180.0; /* (deg/s) */

        for (k = 0; k < MAXCODE; k++)
        {
            //ssr.pbias[k]=pbias[k];
            //ssr.stdpb[k]=(float)stdpb[k];
        }

        for (loc = 0; loc < nav->ns; ++loc)
        {
            if (nav->ssr[loc].sat == sat)
            {
                break;
            }
        }
        if (loc < nav->ns)
        {
            nav->ssr[loc] = ssr;
        }
        else if (loc == nav->ns && loc < MAXEPH)
        {
            nav->ssr[nav->ns] = ssr;
            ++nav->ns;
        }
    }
    return 20;
}
#endif

/* decode rtcm ver.3 message -------------------------------------------------*/
int decode_rtcm3(rtcm_t *rtcm, obs_t *obs, nav_t *nav)
{
    int ret = 0, type = rtcm->type = rtcm_getbitu(rtcm->buff, 24, 12);

    trace(3, "decode_rtcm3: len=%3d type=%d\n", rtcm->len, type);
    switch (type)
    {
    case 999:
        ret = decode_type999(rtcm, obs);
        break;
    case 1001:
        ret = decode_type1001(rtcm, obs);
        break; /* not supported */
    case 1002:
        ret = decode_type1002(rtcm, obs);
        break;
    case 1003:
        ret = decode_type1003(rtcm, obs);
        break; /* not supported */
    case 1004:
        ret = decode_type1004(rtcm, obs);
        break;
    case 1005:
        ret = decode_type1005(rtcm, obs);
        break;
    case 1006:
        ret = decode_type1006(rtcm, obs);
        break;
    case 1007:
        ret = decode_type1007(rtcm, obs);
        break;
    case 1008:
        ret = decode_type1008(rtcm, obs);
        break;
    case 1009:
        ret = decode_type1009(rtcm, obs);
        break; /* not supported */
    case 1010:
        ret = decode_type1010(rtcm, obs);
        break;
    case 1011:
        ret = decode_type1011(rtcm, obs);
        break; /* not supported */
    case 1012:
        ret = decode_type1012(rtcm, obs);
        break;
    case 1013:
        ret = decode_type1013(rtcm);
        break; /* not supported */
    case 1019:
        ret = decode_type1019(rtcm, nav);
        break;
    case 1020:
        ret = decode_type1020(rtcm, nav);
        break;
    case 1021:
        ret = decode_type1021(rtcm);
        break; /* not supported */
    case 1022:
        ret = decode_type1022(rtcm);
        break; /* not supported */
    case 1023:
        ret = decode_type1023(rtcm);
        break; /* not supported */
    case 1024:
        ret = decode_type1024(rtcm);
        break; /* not supported */
    case 1025:
        ret = decode_type1025(rtcm);
        break; /* not supported */
    case 1026:
        ret = decode_type1026(rtcm);
        break; /* not supported */
    case 1027:
        ret = decode_type1027(rtcm);
        break; /* not supported */
    case 1029:
        ret = decode_type1029(rtcm);
        break;
    case 1030:
        ret = decode_type1030(rtcm);
        break; /* not supported */
    case 1031:
        ret = decode_type1031(rtcm);
        break; /* not supported */
    case 1032:
        ret = decode_type1032(rtcm);
        break; /* not supported */
    case 1033:
        ret = decode_type1033(rtcm, obs);
        break;
    case 1034:
        ret = decode_type1034(rtcm);
        break; /* not supported */
    case 1035:
        ret = decode_type1035(rtcm);
        break; /* not supported */
    case 1037:
        ret = decode_type1037(rtcm);
        break; /* not supported */
    case 1038:
        ret = decode_type1038(rtcm);
        break; /* not supported */
    case 1039:
        ret = decode_type1039(rtcm);
        break; /* not supported */
    case 1044:
        ret = decode_type1044(rtcm, nav);
        break;
    case 1045:
        ret = decode_type1045(rtcm, nav);
        break;
    case 1046:
        ret = decode_type1046(rtcm, nav);
        break;
    case 63:
        ret = decode_type1042(rtcm, nav);
        break; /* RTCM draft */
    case 1042:
        ret = decode_type1042(rtcm, nav);
        break;
    case 4001:
        ret = decode_type4001(rtcm);
        //osSemaphoreRelease(rtcm_sem);
        break;
#ifdef _USE_PPP_
    case 1057:
        ret = decode_ssr1(rtcm, _SYS_GPS_, nav, obs);
        break;
    case 1058:
        ret = decode_ssr2(rtcm, _SYS_GPS_, nav, obs);
        break;
    case 1059:
        ret = decode_ssr3(rtcm, _SYS_GPS_, nav, obs);
        break;
    case 1060:
        ret = decode_ssr4(rtcm, _SYS_GPS_, nav, obs);
        break;
    case 1061:
        ret = decode_ssr5(rtcm, _SYS_GPS_, nav);
        break;
    case 1062:
        ret = decode_ssr6(rtcm, _SYS_GPS_, nav);
        break;
    case 1063:
        ret = decode_ssr1(rtcm, _SYS_GLO_, nav, obs);
        break;
    case 1064:
        ret = decode_ssr2(rtcm, _SYS_GLO_, nav, obs);
        break;
    case 1065:
        ret = decode_ssr3(rtcm, _SYS_GLO_, nav, obs);
        break;
    case 1066:
        ret = decode_ssr4(rtcm, _SYS_GLO_, nav, obs);
        break;
    case 1067:
        ret = decode_ssr5(rtcm, _SYS_GLO_, nav);
        break;
    case 1068:
        ret = decode_ssr6(rtcm, _SYS_GLO_, nav);
        break;
#endif
    case 1071:
        ret = decode_msm0(rtcm, obs, _SYS_GPS_);
        break; /* not supported */
    case 1072:
        ret = decode_msm0(rtcm, obs, _SYS_GPS_);
        break; /* not supported */
    case 1073:
        ret = decode_msm0(rtcm, obs, _SYS_GPS_);
        break; /* not supported */
    case 1074:
        ret = decode_msm4(rtcm, obs, _SYS_GPS_);
        break;
    case 1075:
        ret = decode_msm5(rtcm, obs, _SYS_GPS_);
        break;
    case 1076:
        ret = decode_msm6(rtcm, obs, _SYS_GPS_);
        break;
    case 1077:
        ret = decode_msm7(rtcm, obs, _SYS_GPS_);
        break;
    case 1081:
        ret = decode_msm0(rtcm, obs, _SYS_GLO_);
        break; /* not supported */
    case 1082:
        ret = decode_msm0(rtcm, obs, _SYS_GLO_);
        break; /* not supported */
    case 1083:
        ret = decode_msm0(rtcm, obs, _SYS_GLO_);
        break; /* not supported */
    case 1084:
        ret = decode_msm4(rtcm, obs, _SYS_GLO_);
        break;
    case 1085:
        ret = decode_msm5(rtcm, obs, _SYS_GLO_);
        break;
    case 1086:
        ret = decode_msm6(rtcm, obs, _SYS_GLO_);
        break;
    case 1087:
        ret = decode_msm7(rtcm, obs, _SYS_GLO_);
        break;
    case 1091:
        ret = decode_msm0(rtcm, obs, _SYS_GAL_);
        break; /* not supported */
    case 1092:
        ret = decode_msm0(rtcm, obs, _SYS_GAL_);
        break; /* not supported */
    case 1093:
        ret = decode_msm0(rtcm, obs, _SYS_GAL_);
        break; /* not supported */
    case 1094:
        ret = decode_msm4(rtcm, obs, _SYS_GAL_);
        break;
    case 1095:
        ret = decode_msm5(rtcm, obs, _SYS_GAL_);
        break;
    case 1096:
        ret = decode_msm6(rtcm, obs, _SYS_GAL_);
        break;
    case 1097:
        ret = decode_msm7(rtcm, obs, _SYS_GAL_);
        break;
    case 1101:
        ret = decode_msm0(rtcm, obs, _SYS_SBS_);
        break; /* not supported */
    case 1102:
        ret = decode_msm0(rtcm, obs, _SYS_SBS_);
        break; /* not supported */
    case 1103:
        ret = decode_msm0(rtcm, obs, _SYS_SBS_);
        break; /* not supported */
    case 1104:
        ret = decode_msm4(rtcm, obs, _SYS_SBS_);
        break;
    case 1105:
        ret = decode_msm5(rtcm, obs, _SYS_SBS_);
        break;
    case 1106:
        ret = decode_msm6(rtcm, obs, _SYS_SBS_);
        break;
    case 1107:
        ret = decode_msm7(rtcm, obs, _SYS_SBS_);
        break;
    case 1111:
        ret = decode_msm0(rtcm, obs, _SYS_QZS_);
        break; /* not supported */
    case 1112:
        ret = decode_msm0(rtcm, obs, _SYS_QZS_);
        break; /* not supported */
    case 1113:
        ret = decode_msm0(rtcm, obs, _SYS_QZS_);
        break; /* not supported */
    case 1114:
        ret = decode_msm4(rtcm, obs, _SYS_QZS_);
        break;
    case 1115:
        ret = decode_msm5(rtcm, obs, _SYS_QZS_);
        break;
    case 1116:
        ret = decode_msm6(rtcm, obs, _SYS_QZS_);
        break;
    case 1117:
        ret = decode_msm7(rtcm, obs, _SYS_QZS_);
        break;
    case 1121:
        ret = decode_msm0(rtcm, obs, _SYS_BDS_);
        break; /* not supported */
    case 1122:
        ret = decode_msm0(rtcm, obs, _SYS_BDS_);
        break; /* not supported */
    case 1123:
        ret = decode_msm0(rtcm, obs, _SYS_BDS_);
        break; /* not supported */
    case 1124:
        ret = decode_msm4(rtcm, obs, _SYS_BDS_);
        break;
    case 1125:
        ret = decode_msm5(rtcm, obs, _SYS_BDS_);
        break;
    case 1126:
        ret = decode_msm6(rtcm, obs, _SYS_BDS_);
        break;
    case 1127:
        ret = decode_msm7(rtcm, obs, _SYS_BDS_);
        break;
    case 1230:
        ret = decode_type1230(rtcm);
        break; /* not supported */
#ifdef _USE_PPP_
    case 1240:
        ret = decode_ssr1(rtcm, _SYS_GAL_, nav, obs);
        break;
    case 1241:
        ret = decode_ssr2(rtcm, _SYS_GAL_, nav, obs);
        break;
    case 1242:
        ret = decode_ssr3(rtcm, _SYS_GAL_, nav, obs);
        break;
    case 1243:
        ret = decode_ssr4(rtcm, _SYS_GAL_, nav, obs);
        break;
    case 1244:
        ret = decode_ssr5(rtcm, _SYS_GAL_, nav);
        break;
    case 1245:
        ret = decode_ssr6(rtcm, _SYS_GAL_, nav);
        break;
    case 1246:
        ret = decode_ssr1(rtcm, _SYS_QZS_, nav, obs);
        break;
    case 1247:
        ret = decode_ssr2(rtcm, _SYS_QZS_, nav, obs);
        break;
    case 1248:
        ret = decode_ssr3(rtcm, _SYS_QZS_, nav, obs);
        break;
    case 1249:
        ret = decode_ssr4(rtcm, _SYS_QZS_, nav, obs);
        break;
    case 1250:
        ret = decode_ssr5(rtcm, _SYS_QZS_, nav);
        break;
    case 1251:
        ret = decode_ssr6(rtcm, _SYS_QZS_, nav);
        break;
    case 1252:
        ret = decode_ssr1(rtcm, _SYS_SBS_, nav, obs);
        break;
    case 1253:
        ret = decode_ssr2(rtcm, _SYS_SBS_, nav, obs);
        break;
    case 1254:
        ret = decode_ssr3(rtcm, _SYS_SBS_, nav, obs);
        break;
    case 1255:
        ret = decode_ssr4(rtcm, _SYS_SBS_, nav, obs);
        break;
    case 1256:
        ret = decode_ssr5(rtcm, _SYS_SBS_, nav);
        break;
    case 1257:
        ret = decode_ssr6(rtcm, _SYS_SBS_, nav);
        break;
    case 1258:
        ret = decode_ssr1(rtcm, _SYS_BDS_, nav, obs);
        break;
    case 1259:
        ret = decode_ssr2(rtcm, _SYS_BDS_, nav, obs);
        break;
    case 1260:
        ret = decode_ssr3(rtcm, _SYS_BDS_, nav, obs);
        break;
    case 1261:
        ret = decode_ssr4(rtcm, _SYS_BDS_, nav, obs);
        break;
    case 1262:
        ret = decode_ssr5(rtcm, _SYS_BDS_, nav);
        break;
    case 1263:
        ret = decode_ssr6(rtcm, _SYS_BDS_, nav);
        break;
    case 11:
        ret = decode_ssr7(rtcm, _SYS_GLO_, nav);
        break; /* tentative */
    case 12:
        ret = decode_ssr7(rtcm, _SYS_GAL_, nav);
        break; /* tentative */
    case 13:
        ret = decode_ssr7(rtcm, _SYS_QZS_, nav);
        break; /* tentative */
    case 14:
        ret = decode_ssr7(rtcm, _SYS_BDS_, nav);
        break; /* tentative */
#endif
    default:
        ret = 0;
    }
    return ret;
}

/* input rtcm 3 message from stream --------------------------------------------
* fetch next rtcm 3 message and input a message from byte stream
* args   : rtcm_t *rtcm IO   rtcm control struct
*          unsigned char data I stream data (1 byte)
* return : status (-1: error message, 0: no message, 1: input observation data,
*                  2: input ephemeris, 5: input station pos/ant parameters,
*                  10: input ssr messages)
* notes  : before firstly calling the function, time in rtcm control struct has
*          to be set to the approximate time within 1/2 week in order to resolve
*          ambiguity of time in rtcm messages.
*          
*          to specify input options, set rtcm->opt to the following option
*          strings separated by spaces.
*
*          -EPHALL  : input all ephemerides
*          -STA=nnn : input only message with STAID=nnn
*          -GLss    : select signal ss for GPS MSM (ss=1C,1P,...)
*          -RLss    : select signal ss for GLO MSM (ss=1C,1P,...)
*          -ELss    : select signal ss for GAL MSM (ss=1C,1B,...)
*          -JLss    : select signal ss for QZS MSM (ss=1C,2C,...)
*          -CLss    : select signal ss for BDS MSM (ss=2I,7I,...)
*
*          supported RTCM 3 messages
*             (ref [2][3][4][5][6][7][8][9][10][11][12][13][14][15][16][17])
*
*            TYPE       GPS     GLOASS    GALILEO    QZSS     BEIDOU     SBAS
*         ----------------------------------------------------------------------
*          OBS C-L1  : 1001~     1009~       -         -         -         -
*              F-L1  : 1002      1010        -         -         -         -
*              C-L12 : 1003~     1011~       -         -         -         -
*              F-L12 : 1004      1012        -         -         -         -
*
*          NAV       : 1019      1020      1045      1044      1042        -
*                        -         -       1046        -         63*       -
*
*          MSM 1     : 1071~     1081~     1091~     1111~     1121~     1101~
*              2     : 1072~     1082~     1092~     1112~     1122~     1102~
*              3     : 1073~     1083~     1093~     1113~     1123~     1103~
*              4     : 1074      1084      1094      1114      1124      1104
*              5     : 1075      1085      1095      1115      1125      1105
*              6     : 1076      1086      1096      1116      1126      1106
*              7     : 1077      1087      1097      1117      1127      1107
*
*          SSR OBT   : 1057      1063      1240*     1246*     1258*       -
*              CLK   : 1058      1064      1241*     1247*     1259*       -
*              BIAS  : 1059      1065      1242*     1248*     1260*       -
*              OBTCLK: 1060      1066      1243*     1249*     1261*       -
*              URA   : 1061      1067      1244*     1250*     1262*       -
*              HRCLK : 1062      1068      1245*     1251*     1263*       -
*
*          ANT INFO  : 1005 1006 1007 1008 1033
*         ----------------------------------------------------------------------
*                                                    (* draft, ~ only encode)
*
*          for MSM observation data with multiple signals for a frequency,
*          a signal is selected according to internal priority. to select
*          a specified signal, use the input options.
*
*          rtcm3 message format:
*            +----------+--------+-----------+--------------------+----------+
*            | preamble | 000000 |  length   |    data message    |  parity  |
*            +----------+--------+-----------+--------------------+----------+
*            |<-- 8 --->|<- 6 -->|<-- 10 --->|<--- length x 8 --->|<-- 24 -->|
*            
*-----------------------------------------------------------------------------*/

extern uint8_t stnID;
extern uint8_t debug_com_log_on;
extern client_s driver_data_client;
void fill_base_data(rtcm_t *rtcm,int rtcm_len)
{
    uint8_t base_data_buf[2000] = {0};
    double gga_time = get_gnss_time();
    //  sizeof(",%02x\r\n") 5 sizeof(',') 1
    uint32_t data_len = rtcm_len + 5*sizeof(char) + 1*sizeof(char); 
    int head_len = sprintf(( char*)base_data_buf,"$GPREF,%6.2f,%04lu,", gga_time, data_len);
    memcpy(base_data_buf + strlen(( char*)base_data_buf),rtcm->buff,rtcm_len);
    int all_bytes_to_sum = head_len + rtcm_len;
    char sum = 0;
    for(int i = 0;i < all_bytes_to_sum;i++)
    {
        sum ^= base_data_buf[i];
    }
    int end_len = sprintf((char*)base_data_buf + head_len + rtcm_len,",%02x\r\n",sum);
    if (debug_com_log_on) {
       uart_write_bytes(UART_DEBUG,( char*)base_data_buf,head_len + rtcm_len + end_len,1);
    }
    if(get_tcp_data_driver_state() == CLIENT_STATE_INTERACTIVE)
    {
        //driver_data_push((char*)base_data_buf,head_len + rtcm_len + end_len);
        client_write_data(&driver_data_client, (const uint8_t *)base_data_buf,head_len + rtcm_len + end_len,0x01);
    }
}

uint8_t rtcm_decode_completion = 0;
uint32_t rtcm_decode_length = 0;

extern int input_rtcm3_data(rtcm_t *rtcm, unsigned char data, obs_t *obs, nav_t *nav)
{
    /* synchronize frame */
    int ret = 0;
    rtcm->type = 0;
    if (rtcm->nbyte == 0)
    {
        /* key = 0 => RTCM, key = 1 => NMEA */
        if (data == RTCM3PREAMB /* RTCM data format */
                                //  || data == '$'      /* NMEA data format */
        )
        {
            rtcm->key = data;
            rtcm->buff[rtcm->nbyte++] = data;
        }
        return 0;
    }

    /* RTCM decorder */
    rtcm->buff[rtcm->nbyte++] = data;

    if (rtcm->nbyte == 3)
    {
        rtcm->len = rtcm_getbitu(rtcm->buff, 14, 10) + 3; /* length without parity */
    }
    if (rtcm->nbyte < 3 || rtcm->nbyte < rtcm->len + 3)
        return 0;
#ifdef DEBUG_ALL
    if((rtcm->nbyte > 0) && (stnID == BASE))
    {
        fill_base_data(rtcm,rtcm->nbyte);
    }
#endif
    rtcm_decode_length = rtcm->nbyte;
    rtcm->nbyte = 0;
    rtcm->type = rtcm_getbitu(rtcm->buff, 24, 12);

    /* check parity */
    if (rtk_crc24q(rtcm->buff, rtcm->len) != rtcm_getbitu(rtcm->buff, rtcm->len * 8, 24))
    {
        trace(2, "rtcm3 parity error: len=%d\n", rtcm->len);
        return 0;
    }

    rtcm_decode_completion = 1;

    // printf("rtcm type: %d\r\n", rtcm->type);

    /* decode rtcm3 message */
    return decode_rtcm3(rtcm, obs, nav);
}

extern int input_rtcm3(unsigned char data, unsigned int stnID, gnss_rtcm_t *gnss)
{
    rtcm_t *rtcm = NULL;
    obs_t *obs = NULL;
    nav_t *nav = NULL;
    int ret = 0;
    static int obs_flag = 0;

    if (stnID < MAXSTN)
    {
        rtcm = gnss->rcv + stnID;
        nav = &gnss->nav;
        obs = gnss->obs + stnID;
        ret = input_rtcm3_data(rtcm, data, obs, nav);

        if (stnID == BASE && rtcm->time.time == 0) {
            rtcm->time.time = gnss->rcv[ROVER].time.time;
        }
    }

    return ret;
}

