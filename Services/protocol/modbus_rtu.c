#include "modbus_rtu.h"
#include <stddef.h>

uint16_t mb_crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
        }
    }
    return crc;
}

int mb_check_crc(const uint8_t *frame, uint16_t len)
{
    if (frame == NULL || len < 5) return -1;
    uint16_t recv = (frame[len - 1] << 8) | frame[len - 2];
    return (mb_crc16(frame, len - 2) == recv) ? 0 : -1;
}

/* ── 上位机响应帧 ── */
int mb_build_rsp_read(uint8_t *tx, uint16_t max_len,
                       uint8_t id, const uint16_t *regs, uint16_t count)
{
    uint16_t need = 5 + count * 2;
    if (tx == NULL || max_len < need) return -1;
    tx[0] = id;  tx[1] = MB_FC_READ_HOLDING;  tx[2] = (uint8_t)(count * 2);
    for (uint16_t i = 0; i < count; i++) {
        tx[3 + i * 2]     = (uint8_t)(regs[i] >> 8);
        tx[3 + i * 2 + 1] = (uint8_t)(regs[i] & 0xFF);
    }
    uint16_t crc = mb_crc16(tx, 3 + count * 2);
    tx[3 + count * 2]     = (uint8_t)(crc & 0xFF);
    tx[3 + count * 2 + 1] = (uint8_t)(crc >> 8);
    return 3 + count * 2 + 2;
}

int mb_build_rsp_write_single(uint8_t *tx, uint16_t max_len,
                               uint8_t id, uint16_t addr, uint16_t val)
{
    if (tx == NULL || max_len < 8) return -1;
    tx[0] = id;  tx[1] = MB_FC_WRITE_SINGLE;
    tx[2] = (uint8_t)(addr >> 8);  tx[3] = (uint8_t)(addr & 0xFF);
    tx[4] = (uint8_t)(val >> 8);   tx[5] = (uint8_t)(val & 0xFF);
    uint16_t crc = mb_crc16(tx, 6);
    tx[6] = (uint8_t)(crc & 0xFF);  tx[7] = (uint8_t)(crc >> 8);
    return 8;
}

int mb_build_rsp_write_multi(uint8_t *tx, uint16_t max_len,
                              uint8_t id, uint16_t addr, uint16_t count)
{
    if (tx == NULL || max_len < 8) return -1;
    tx[0] = id;  tx[1] = MB_FC_WRITE_MULTI;
    tx[2] = (uint8_t)(addr >> 8);  tx[3] = (uint8_t)(addr & 0xFF);
    tx[4] = (uint8_t)(count >> 8); tx[5] = (uint8_t)(count & 0xFF);
    uint16_t crc = mb_crc16(tx, 6);
    tx[6] = (uint8_t)(crc & 0xFF); tx[7] = (uint8_t)(crc >> 8);
    return 8;
}

int mb_build_rsp_exc(uint8_t *tx, uint16_t max_len,
                      uint8_t id, uint8_t exc_code)
{
    if (tx == NULL || max_len < 5) return -1;
    tx[0] = id;  tx[1] = 0x83;  tx[2] = exc_code;
    uint16_t crc = mb_crc16(tx, 3);
    tx[3] = (uint8_t)(crc & 0xFF);  tx[4] = (uint8_t)(crc >> 8);
    return 5;
}

/* ── DSP 请求帧 ── */
int mb_build_req_read(uint8_t *tx, uint16_t max_len,
                       uint8_t id, uint16_t addr, uint16_t count)
{
    if (tx == NULL || max_len < 8) return -1;
    tx[0] = id;  tx[1] = MB_FC_READ_HOLDING;
    tx[2] = (uint8_t)(addr >> 8);  tx[3] = (uint8_t)(addr & 0xFF);
    tx[4] = (uint8_t)(count >> 8); tx[5] = (uint8_t)(count & 0xFF);
    uint16_t crc = mb_crc16(tx, 6);
    tx[6] = (uint8_t)(crc & 0xFF); tx[7] = (uint8_t)(crc >> 8);
    return 8;
}

int mb_build_req_write_single(uint8_t *tx, uint16_t max_len,
                                uint8_t id, uint16_t addr, uint16_t val)
{
    if (tx == NULL || max_len < 8) return -1;
    tx[0] = id;  tx[1] = MB_FC_WRITE_SINGLE;
    tx[2] = (uint8_t)(addr >> 8);  tx[3] = (uint8_t)(addr & 0xFF);
    tx[4] = (uint8_t)(val >> 8);   tx[5] = (uint8_t)(val & 0xFF);
    uint16_t crc = mb_crc16(tx, 6);
    tx[6] = (uint8_t)(crc & 0xFF); tx[7] = (uint8_t)(crc >> 8);
    return 8;
}

int mb_build_req_write_multi(uint8_t *tx, uint16_t max_len,
                              uint8_t id, uint16_t addr,
                              const uint16_t *vals, uint16_t count)
{
    uint16_t need = 7 + count * 2 + 2;
    if (tx == NULL || max_len < need || vals == NULL) return -1;
    tx[0] = id;  tx[1] = MB_FC_WRITE_MULTI;
    tx[2] = (uint8_t)(addr >> 8);  tx[3] = (uint8_t)(addr & 0xFF);
    tx[4] = (uint8_t)(count >> 8); tx[5] = (uint8_t)(count & 0xFF);
    tx[6] = (uint8_t)(count * 2);
    for (uint16_t i = 0; i < count; i++) {
        tx[7 + i * 2]     = (uint8_t)(vals[i] >> 8);
        tx[7 + i * 2 + 1] = (uint8_t)(vals[i] & 0xFF);
    }
    uint16_t crc = mb_crc16(tx, 7 + count * 2);
    tx[7 + count * 2]     = (uint8_t)(crc & 0xFF);
    tx[7 + count * 2 + 1] = (uint8_t)(crc >> 8);
    return 7 + count * 2 + 2;
}

/* ── 响应解析 ── */
int mb_parse_rsp_ok(const uint8_t *frame, uint16_t len)
{
    if (frame == NULL || len < 5) return -1;
    return mb_check_crc(frame, len);
}

int mb_parse_rsp_read(const uint8_t *frame, uint16_t len,
                       uint16_t *out_count, uint16_t *out_vals)
{
    if (frame == NULL || len < 3 || mb_check_crc(frame, len) != 0) return -1;
    uint8_t bc = frame[2];
    if (bc != len - 5) return -1;
    uint16_t n = bc / 2;
    if (out_vals != NULL && n > 0) {
        for (uint16_t i = 0; i < n; i++) {
            out_vals[i] = (uint16_t)((frame[3 + i * 2] << 8) | frame[3 + i * 2 + 1]);
        }
    }
    if (out_count != NULL) *out_count = n;
    return (int)n;
}

int mb_parse_rsp_write_single(const uint8_t *frame, uint16_t len,
                               uint16_t *out_addr, uint16_t *out_val)
{
    if (frame == NULL || len < 8 || mb_check_crc(frame, len) != 0) return -1;
    if (out_addr != NULL) *out_addr = (uint16_t)((frame[2] << 8) | frame[3]);
    if (out_val  != NULL) *out_val  = (uint16_t)((frame[4] << 8) | frame[5]);
    return 0;
}

int mb_parse_rsp_write_multi(const uint8_t *frame, uint16_t len,
                              uint16_t *out_addr, uint16_t *out_count)
{
    if (frame == NULL || len < 8 || mb_check_crc(frame, len) != 0) return -1;
    if (out_addr  != NULL) *out_addr  = (uint16_t)((frame[2] << 8) | frame[3]);
    if (out_count != NULL) *out_count = (uint16_t)((frame[4] << 8) | frame[5]);
    return 0;
}
