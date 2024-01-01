#ifndef INC_MAINAPP_H
#define INC_MAINAPP_H

#include "common_defines.h"

#define PMB_CAN_HB_MSG_INTVL       (500)
#define PMB_CAN_BATT_MSG_INTVL     (200)
#define PMB_CAN_BOARD_MSG_INTVL    (1000)
#define PMB_OLED_REFRESH_INTVL     (500)
#define PMB_STATUS_UPDATE_INTVL    (50)        // Internal Data Update

// 
// BQ calibration
// 
#define PMB_PERFORM_CALIBRATION         (0)
#define PMB_CALIBRATION_VOLTAGE         (16000)
#define PMB_CALIBRATION_CURRENT         (3000)

#endif // INC_MAINAPP_H