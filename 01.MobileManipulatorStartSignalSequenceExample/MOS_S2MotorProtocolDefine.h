#ifndef MOS_S2_MOTOR_PROTOCOL_DEFINE_H
#define MOS_S2_MOTOR_PROTOCOL_DEFINE_H

// HEX로 표기됨
#define INST_RESET_DATA                   0x01
#define SIZE_RESET_DATA                   0x06

#define INST_REBOOT                       0x02
#define SIZE_REBOOT                       0x06

#define INST_GET_STATUS                   0x05
#define SIZE_GET_STATUS                   0x06

#define INST_SET_ID                       0x06
#define SIZE_SET_ID                       0x07

#define INST_SET_PID_NO_SAVE              0x47
#define SIZE_SET_PID_NO_SAVE              0x09

#define INST_SET_POSITION_OR_WHEEL        0x09
#define SIZE_SET_POSITION_OR_WHEEL        0x08

#define INST_SET_OFFSET                   0x0A
#define SIZE_SET_OFFSET                   0x07

#define INST_SET_MARGIN                   0x0C
#define SIZE_SET_MARGIN                   0x07

#define INST_SET_POSITION_LIMIT           0x0F
#define SIZE_SET_POSITION_LIMIT           0x0A

#define INST_SET_TORQUE                   0x10
#define SIZE_SET_TORQUE                   0x07

#define INST_SET_LED                      0x11
#define SIZE_SET_LED                      0x07

#define INST_GET_PID                      0x12
#define SIZE_GET_PID                      0x06

#define INST_GET_TEMPERATURE              0x13
#define SIZE_GET_TEMPERATURE              0x06

#define INST_GET_POSITION                 0x14
#define SIZE_GET_POSITION                 0x06

#define INST_GET_OFFSET                   0x15
#define SIZE_GET_OFFSET                   0x06

#define INST_GET_MARGIN                   0x17
#define SIZE_GET_MARGIN                   0x06

#define INST_GET_POSITION_LIMIT           0x1A
#define SIZE_GET_POSITION_LIMIT           0x06

#define INST_SET_SYNC                     0x20
#define SIZE_SET_SYNC                     0x06

#define INST_SET_NEXT_POSITION_OR_WHEEL   0x21
#define SIZE_SET_NEXT_POSITION_OR_WHEEL   0x08

#define INST_SET_BAUDRATE                 0x22
#define SIZE_SET_BAUDRATE                 0x07

#define INST_SET_ZERO_POSITION            0x23
#define SIZE_SET_ZERO_POSITION            0x06

#define INST_KINETIC_MEMORY_CPS           0x40
#define SIZE_KINETIC_MEMORY_CPS           0x07

#define INST_KINETIC_MEMORY_SET_FLAG      0x41
#define SIZE_KINETIC_MEMORY_SET_FLAG      0x07

#define INST_KINETIC_MEMORY_GET_SIZE      0x44
#define SIZE_KINETIC_MEMORY_GET_SIZE      0x06

#define ACK_GET_STATUS                    0x85
#define ACK_GET_PID                       0x92
#define ACK_GET_TEMPERATURE               0x93
#define ACK_GET_POSITION                  0x94
#define ACK_GET_OFFSET                    0x95
#define ACK_GET_MARGIN                    0x97
#define ACK_GET_POSITION_LIMIT            0x9A
#define ACK_KINETIC_MEMORY_GET_SIZE       0xC4

#endif
