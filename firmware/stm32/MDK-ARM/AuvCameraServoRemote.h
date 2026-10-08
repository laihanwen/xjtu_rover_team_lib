#ifndef AUV_CAMERA_SERVO_REMOTE_H
#define AUV_CAMERA_SERVO_REMOTE_H
#include "AuvCameraServo.h"

/* SC down: hold; middle: camera; upper: removed main servo placeholder. */
static inline void AuvCameraServo_SelectRemote(uint8_t selector, uint8_t dial,
                                               uint8_t dial_valid, uint8_t armed)
{
    AuvCameraServo_CommandRemote(dial,
        (selector == 1U && dial_valid == 1U) ? 1U : 0U, armed);
}
#endif
