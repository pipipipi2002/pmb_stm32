#ifndef INC_CAN_DEFINES_H
#define INC_CAN_DEFINES_H

#define BB_CAN_STD_MSG_SIZE                 (8)
#define BB_CAN_HB_MSG_SIZE                  (1)

/*
 * CAN Message IDs
 */
#define BB_CAN_ID_THR_1_CTRL                (0)
#define BB_CAN_ID_THR_2_CTRL                (1)
#define BB_CAN_ID_ACT_CTRL                  (2)
#define BB_CAN_ID_PWR_CTRL                  (3)
#define BB_CAN_ID_HEARTBEAT                 (4)
#define BB_CAN_ID_TEMP_SBC                  (5)

#define BB_CAN_ID_STB_STAT                  (11)
#define BB_CAN_ID_LED_CTRL                  (12)
#define BB_CAN_ID_BUTTON_STAT               (13)

#define BB_CAN_ID_THR_1_RPM                 (17)
#define BB_CAN_ID_THR_2_RPM                 (18)
#define BB_CAN_ID_THR_1_DUTY                (19)
#define BB_CAN_ID_THR_2_DUTY                (20)

#define BB_CAN_ID_BATT_1_STAT               (23)
#define BB_CAN_ID_PMB_1_STAT                (24)
#define BB_CAN_ID_BATT_2_STAT               (25)
#define BB_CAN_ID_PMB_2_STAT                (26)

#define BB_CAN_ID_PWR_STAT                  (30)

#define BB_CAN_ID_ERR_TAB                   (32)
#define BB_CAN_ID_ERR_STB                   (33)
#define BB_CAN_ID_ERR_PMB_1                 (34)
#define BB_CAN_ID_ERR_PMB_2                 (35)
#define BB_CAN_ID_ERR_SBCCAN                (36)

/*
 * Heartbeat IDs
 */
#define BB_HEARTBEAT_ID_SBC                 (1)
#define BB_HEARTBEAT_ID_SBCCAN              (2)
#define BB_HEARTBEAT_ID_TAB                 (4)
#define BB_HEARTBEAT_ID_STB                 (5)
#define BB_HEARTBEAT_ID_ACT                 (6)
#define BB_HEARTBEAT_ID_PMB_1               (7)
#define BB_HEARTBEAT_ID_PMB_2               (8)

#endif // INC_CAN_DEFINES_H