/**
 * Unit tests for MSP frame encoding
 *
 * Uses mock UART to capture transmitted bytes and verify frame structure.
 */

#include "unity.h"
#include "msp.h"
#include "mock_uart.h"

void setUp(void)
{
    mock_uart_reset();
    /* msp_init() calls uart_init + msp_dp_options, which sends a frame.
     * We init once then reset the capture for each test. */
    msp_init();
    mock_uart_reset();
}

void tearDown(void) {}

/* ---- Helper: get captured UART_AUX (port 6) data ---- */

static const uint8_t *get_tx(uint16_t *len)
{
    *len = mock_uart_tx[UART_AUX].len;
    return mock_uart_tx[UART_AUX].buf;
}

/* ---- Frame structure ---- */

void test_frame_header(void)
{
    uint8_t payload[] = { 0x42 };
    msp_send(99, payload, 1);

    uint16_t len;
    const uint8_t *tx = get_tx(&len);

    TEST_ASSERT_TRUE(len >= 6);
    TEST_ASSERT_EQUAL_UINT8('$', tx[0]);
    TEST_ASSERT_EQUAL_UINT8('M', tx[1]);
    TEST_ASSERT_EQUAL_UINT8('>', tx[2]);
}

void test_frame_length_and_cmd(void)
{
    uint8_t payload[] = { 0xAA, 0xBB };
    msp_send(42, payload, 2);

    uint16_t len;
    const uint8_t *tx = get_tx(&len);

    TEST_ASSERT_EQUAL_UINT8(2, tx[3]);   /* length = payload bytes */
    TEST_ASSERT_EQUAL_UINT8(42, tx[4]);  /* command ID */
}

void test_frame_crc(void)
{
    uint8_t payload[] = { 0x01, 0x02, 0x03 };
    msp_send(MSP_DISPLAYPORT, payload, 3);

    uint16_t len;
    const uint8_t *tx = get_tx(&len);

    /* CRC = XOR of len, cmd, payload bytes */
    uint8_t expected_crc = 3 ^ MSP_DISPLAYPORT ^ 0x01 ^ 0x02 ^ 0x03;
    TEST_ASSERT_EQUAL_UINT8(expected_crc, tx[len - 1]);
}

void test_frame_total_length(void)
{
    uint8_t payload[] = { 0xDE, 0xAD };
    msp_send(100, payload, 2);

    uint16_t len;
    get_tx(&len);

    /* $M> (3) + len (1) + cmd (1) + payload (2) + crc (1) = 8 */
    TEST_ASSERT_EQUAL_UINT16(8, len);
}

/* ---- Empty payload ---- */

void test_empty_payload(void)
{
    msp_send(10, NULL, 0);

    uint16_t len;
    const uint8_t *tx = get_tx(&len);

    /* $M> + len(0) + cmd(10) + crc */
    TEST_ASSERT_EQUAL_UINT16(6, len);
    TEST_ASSERT_EQUAL_UINT8(0, tx[3]);   /* length = 0 */
    TEST_ASSERT_EQUAL_UINT8(10, tx[4]);  /* cmd */
    /* CRC = 0 ^ 10 = 10 */
    TEST_ASSERT_EQUAL_UINT8(10, tx[5]);
}

/* ---- DisplayPort commands ---- */

void test_dp_clear(void)
{
    msp_dp_clear();

    uint16_t len;
    const uint8_t *tx = get_tx(&len);

    TEST_ASSERT_EQUAL_UINT8(1, tx[3]);              /* length = 1 */
    TEST_ASSERT_EQUAL_UINT8(MSP_DISPLAYPORT, tx[4]); /* cmd */
    TEST_ASSERT_EQUAL_UINT8(MSP_DP_CLEAR, tx[5]);   /* sub-cmd */
}

void test_dp_draw(void)
{
    msp_dp_draw();

    uint16_t len;
    const uint8_t *tx = get_tx(&len);

    TEST_ASSERT_EQUAL_UINT8(MSP_DP_DRAW, tx[5]);
}

void test_dp_write(void)
{
    msp_dp_write(3, 10, MSP_DP_ATTR_WARN, "HI");

    uint16_t len;
    const uint8_t *tx = get_tx(&len);

    /* Payload: subcmd(1) + row(1) + col(1) + attr(1) + "HI"(2) = 6 */
    TEST_ASSERT_EQUAL_UINT8(6, tx[3]);               /* length */
    TEST_ASSERT_EQUAL_UINT8(MSP_DISPLAYPORT, tx[4]); /* cmd */
    TEST_ASSERT_EQUAL_UINT8(MSP_DP_WRITE, tx[5]);    /* sub-cmd */
    TEST_ASSERT_EQUAL_UINT8(3, tx[6]);                /* row */
    TEST_ASSERT_EQUAL_UINT8(10, tx[7]);               /* col */
    TEST_ASSERT_EQUAL_UINT8(MSP_DP_ATTR_WARN, tx[8]); /* attr */
    TEST_ASSERT_EQUAL_UINT8('H', tx[9]);
    TEST_ASSERT_EQUAL_UINT8('I', tx[10]);
}

void test_dp_options(void)
{
    msp_dp_options(0);

    uint16_t len;
    const uint8_t *tx = get_tx(&len);

    /* Payload: subcmd + font + rows + cols_lo + cols_hi = 5 */
    TEST_ASSERT_EQUAL_UINT8(5, tx[3]);
    TEST_ASSERT_EQUAL_UINT8(MSP_DP_OPTIONS, tx[5]);  /* sub-cmd */
    TEST_ASSERT_EQUAL_UINT8(0, tx[6]);                /* font */
    TEST_ASSERT_EQUAL_UINT8(MSP_OSD_ROWS, tx[7]);    /* rows = 18 */
    TEST_ASSERT_EQUAL_UINT8(MSP_OSD_COLS & 0xFF, tx[8]); /* cols lo = 50 */
    TEST_ASSERT_EQUAL_UINT8(0, tx[9]);                /* cols hi = 0 */
}

void test_dp_heartbeat(void)
{
    msp_dp_heartbeat();

    uint16_t len;
    const uint8_t *tx = get_tx(&len);

    TEST_ASSERT_EQUAL_UINT8(MSP_DP_HEARTBEAT, tx[5]);
}

/* ---- CRC correctness for known frame ---- */

void test_crc_known_frame(void)
{
    /* dp_clear: payload = [0x01], cmd = 182 */
    msp_dp_clear();

    uint16_t len;
    const uint8_t *tx = get_tx(&len);

    /* Manually compute: crc = 1 ^ 182 ^ 1 = 182 */
    uint8_t crc = tx[3]; /* len byte */
    crc ^= tx[4];        /* cmd byte */
    for (int i = 5; i < len - 1; i++) {
        crc ^= tx[i];
    }
    TEST_ASSERT_EQUAL_UINT8(crc, tx[len - 1]);
}

/* ---- Runner ---- */

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_frame_header);
    RUN_TEST(test_frame_length_and_cmd);
    RUN_TEST(test_frame_crc);
    RUN_TEST(test_frame_total_length);
    RUN_TEST(test_empty_payload);
    RUN_TEST(test_dp_clear);
    RUN_TEST(test_dp_draw);
    RUN_TEST(test_dp_write);
    RUN_TEST(test_dp_options);
    RUN_TEST(test_dp_heartbeat);
    RUN_TEST(test_crc_known_frame);

    return UNITY_END();
}
