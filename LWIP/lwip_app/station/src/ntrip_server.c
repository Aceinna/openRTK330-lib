#include "ntrip_server.h"
#include "string.h"
#include "osapi.h"
#include "station_tcp.h"

// base station
#define MAX_STATION_POSITION_CALC_TIME 60

uint8_t base_station_run = 0;
double station_calc_pos[3];
uint32_t station_position_calc_time = 0;
uint32_t base_pos_send_divide = 0;

static void pos2ecef(const double *pos, double *r)
{
	double sinp = sin(pos[0]), cosp = cos(pos[0]), sinl = sin(pos[1]);
	double cosl = cos(pos[1]);
	double e2 = FE_WGS84 * (2.0 - FE_WGS84);
	double v = RE_WGS84 / sqrt(1.0 - e2 * sinp * sinp);

	r[0] = (v + pos[2]) * cosp * cosl;
	r[1] = (v + pos[2]) * cosp * sinl;
	r[2] = (v * (1.0 - e2) + pos[2]) * sinp;
}

/** ***************************************************************************
 * @name fill_ntrip_server_request_payload() 
 * @brief
 * @param *payload point to buffer
 *        *payloadLen point to buffer length
 * @retval N/A
 ******************************************************************************/
static void fill_ntrip_server_request_payload(uint8_t *payload, uint16_t *payloadLen)
{
    strcpy((char *)payload, "SOURCE ");
    strcat((char *)payload, (const char *)get_ntrip_server_password());
    strcat((char *)payload, " /");
    strcat((char *)payload, (const char *)get_ntrip_server_mount_point());
    strcat((char *)payload, "\r\n");
    strcat((char *)payload, "Source-Agent: NTRIP AceinnaServer/0.1\r\n");
    strcat((char *)payload, "\r\n");

    *payloadLen = strlen((const char *)payload);
}

uint32_t get_station_position_calc_time(void)
{
    return station_position_calc_time;
}

void base_station_run_update(void)
{
    uint16_t bpt = get_base_position_type(); 

    if (get_station_mode() == MODE_NTRIP_SERVER) {
        station_calc_pos[0] = 0.0;
        station_calc_pos[1] = 0.0;
        station_calc_pos[2] = 0.0;
        station_position_calc_time = 0;
        
        if (bpt == BASE_POSITION_REFERENCE) {
            base_station_run = 1;
        } else if (bpt == BASE_POSITION_SPP) {
            base_station_run = 2;
        } else {
            base_station_run = 3;
        }
    }
}

uint8_t base_station_get_run_status(void)
{
    return base_station_run;
}

uint8_t station_position_calc(double lat, double lon, double height, uint8_t mode)
{
    uint16_t bpt = get_base_position_type();
    if ((bpt == BASE_POSITION_RTK && mode > 1) || bpt == BASE_POSITION_SPP || bpt == BASE_POSITION_AUTO) {
        if (base_station_run > 1 && station_position_calc_time < MAX_STATION_POSITION_CALC_TIME) {
            station_calc_pos[0] += lat;
            station_calc_pos[1] += lon;
            station_calc_pos[2] += height;
            station_position_calc_time++;

#ifdef STATION_TCP_DEBUG
                printf("NTRIP-SERVER: calc %ld: [%.8f %.8f %.3f]\r\n{%.8f %.8f %.3f}!\r\n",
                    station_position_calc_time,
                    station_calc_pos[0]*R2D/station_position_calc_time, station_calc_pos[1]*R2D/station_position_calc_time, station_calc_pos[2]/station_position_calc_time,
                    lat*R2D, lon*R2D, height);
#endif

            if (station_position_calc_time >= MAX_STATION_POSITION_CALC_TIME) {

                station_calc_pos[0] /= MAX_STATION_POSITION_CALC_TIME;
                station_calc_pos[1] /= MAX_STATION_POSITION_CALC_TIME;
                station_calc_pos[2] /= MAX_STATION_POSITION_CALC_TIME;

                double ref_rr[3], rr[3], blh[3];
                pos2ecef((const double*)station_calc_pos, ref_rr);
                blh[0] = get_reference_latitude() * DEG_TO_RAD;
                blh[1] = get_reference_longitude() * DEG_TO_RAD;
                blh[2] = get_reference_height();
                pos2ecef((const double*)blh, rr);

                double distance = sqrt((rr[0] - ref_rr[0]) * (rr[0] - ref_rr[0]) +
                                       (rr[1] - ref_rr[1]) * (rr[1] - ref_rr[1]));
#ifdef STATION_TCP_DEBUG
    printf("NTRIP-SERVER: calc over. distance %lf, %lf %lf %lf %lf %lf %lf\r\n", distance, rr[0],rr[1],rr[2],ref_rr[0],ref_rr[1],ref_rr[2]);
#endif
                uint8_t use_new_pos = 0;
                if (bpt == BASE_POSITION_RTK) {
                    if (distance > 0.2) {
                        use_new_pos = 1;
                    }
                } else {
                    if (distance > 1.5) {
                        use_new_pos = 1;
                    }
                }
                if (use_new_pos) {
#ifdef STATION_TCP_DEBUG
    printf("NTRIP-SERVER: use new pos\r\n");
#endif
                    // save pos or use old pos
                    set_reference_latitude(station_calc_pos[0] * R2D);
                    set_reference_longitude(station_calc_pos[1] * R2D);
                    set_reference_height(station_calc_pos[2]);
                    SaveUserConfig();
                } else {
#ifdef STATION_TCP_DEBUG
    printf("NTRIP-SERVER: use old pos\r\n");
#endif
                    station_calc_pos[0] = get_reference_latitude() * DEG_TO_RAD;
                    station_calc_pos[1] = get_reference_longitude() * DEG_TO_RAD;
                    station_calc_pos[2] = get_reference_height();
                }

                base_station_run = 1;
                netif_station_tcp_config_changed();
            }
            return 1;
        }
    }
    return 0;
}

double* get_station_calc_position(void)
{
    return station_calc_pos;
}

double get_station_pos_lat(void)
{
    return station_calc_pos[0];
}

double get_station_pos_lon(void)
{
    return station_calc_pos[1];
}

double get_station_pos_height(void)
{
    return station_calc_pos[2];
}

/** ***************************************************************************
 * @name encode_rtcm3_1006
 * @brief 
 * @param *buf point to data buffer
 *        len length
 * @retval N/A
 ******************************************************************************/
void encode_rtcm3_1006(uint8_t* buff, uint16_t* len, uint8_t mode)
{
    double blh[3];
    double rr[3];
    int i = 0, j;
    uint32_t crc;
    double anth;

    buff[0] = 0xD3;
    setbitu(buff, 14, 10, 21);
    setbitu(buff, 24, 12, 1006);
    i = 24 + 12;
    setbitu(buff, i, 12, get_station_id()); // reference station id
    i += 12;
    setbitu(buff, i, 6, 0); // itrf realization year
    i += 6 + 4;

    if (mode != BASE_POSITION_REFERENCE) {
        pos2ecef((const double*)get_station_calc_position(), rr);

#ifdef STATION_TCP_DEBUG
    printf("NTRIP-SERVER:[CALCED] %lf %lf %lf\r\n", get_station_pos_lat()*R2D, get_station_pos_lon()*R2D, get_station_pos_height());
#endif
    } else {
        blh[0] = get_reference_latitude() * DEG_TO_RAD;
        blh[1] = get_reference_longitude() * DEG_TO_RAD;
        blh[2] = get_reference_height();
        pos2ecef((const double*)blh, rr);

#ifdef STATION_TCP_DEBUG
    printf("NTRIP-SERVER:[REF] %lf %lf %lf\r\n", get_reference_latitude(), get_reference_longitude(), get_reference_height());
#endif
    }

    for (j = 0; j < 3; j++) {
        rr[j] = rr[j] * 10000;
    }
    rtcm_setbits_38(buff, i, rr[0]);
    i += 38 + 2;
    rtcm_setbits_38(buff, i, rr[1]);
    i += 38 + 2;
    rtcm_setbits_38(buff, i, rr[2]);
    i += 38;
    
    anth = get_antenna_height();
    if (0.0 <= anth && anth <= 6.5535) {
        anth = round(anth/0.0001);
    } else {
        anth = 0;
    }
    setbitu(buff, i, 16, (uint32_t)anth);

    crc = rtk_crc24q(buff, 24);
    setbitu(buff, 24 * 8, 24, crc);

    *len = 27;
}


int32_t ntrip_server_request(struct netconn *conn, NETCONN_STATE *state, uint8_t *buf, uint16_t size)
{
    int32_t res = -1;
    uint16_t len = 0;
    err_t err = ERR_OK;

    memset(buf, 0, size);
    fill_ntrip_server_request_payload(buf, &len);
    err = netconn_write(conn, buf, len, NETCONN_COPY);
    if (!err) {
        OS_Delay(1500);
        err = netconn_read(conn, buf, &len, size-1);
        if (!err) {
            buf[len] = 0;
            if (len && strstr((char *)buf, "ICY 200 OK") != NULL) {
                base_pos_send_divide = 900;
                res = 0;
            } else {
#ifdef STATION_TCP_DEBUG
    printf("NTRIP-SERVER: request fail %s\r\n", buf);
#endif
            }
        }
    }

    netconn_check_ret(err, state);

    return res;
}

void ntrip_server_send_pos(struct netconn *conn, NETCONN_STATE *state)
{
    uint8_t rtcm_buf[64];
    uint16_t rtcm_len;
    err_t err = ERR_OK;

    base_pos_send_divide++;
    if (base_pos_send_divide >= 1000) {
        base_pos_send_divide = 0;
        encode_rtcm3_1006(rtcm_buf, &rtcm_len, get_base_position_type());
        err= netconn_write(conn, rtcm_buf, rtcm_len, NETCONN_NOFLAG);

        netconn_check_ret(err, state);
    }
}
