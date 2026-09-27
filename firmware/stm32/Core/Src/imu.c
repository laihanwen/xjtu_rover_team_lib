#include "main.h"
#include "imu.h"
#include "usart.h"
#include <string.h>

// 在解析函数中添加
#ifdef DEBUG_IMU
    printf("Received frame: TID=%u, Len=%u, Total=%u\n", 
           tid, payload_len, total_len);
    printf("Checksum: calc(%02X,%02X) vs recv(%02X,%02X)\n",
           checksum.ck1, checksum.ck2, received_ck1, received_ck2);
    printf("Data ID: 0x%02X, Data Len: %u\n", data_id, data_len);
    printf("Raw values: pitch=%ld, roll=%ld, yaw=%ld\n", 
           pitch_raw, roll_raw, yaw_raw);
    printf("Euler angles: pitch=%.3f, roll=%.3f, yaw=%.3f\n",
           Angle_Measure.pit, Angle_Measure.rol, Angle_Measure.yaw);
#endif

// 全局变量
FLOAT_Angle Angle_Measure;
FLOAT_Angle preangle;
uint8_t imu_data_ready = 0;

// 陀螺仪漂移补偿相关
FLOAT_Angle imu_offset = {0.0f, 0.0f, 0.0f};  // 存储第一次有效数据的偏移量
uint8_t imu_offset_calibrated = 0;  // 标志位：是否已经校准偏移量

// 修改缓冲区大小为最大帧长度+安全余量
#define H30_MAX_FRAME_LEN 270  // 262 + 8字节余量

// H30接收状态机
static enum {
    H30_STATE_IDLE,
    H30_STATE_HEADER1,
    H30_STATE_HEADER2,
    H30_STATE_RECEIVING
} h30_state = H30_STATE_IDLE;

static uint8_t h30_rx_buffer[H30_MAX_FRAME_LEN];
static uint16_t h30_rx_index = 0;
static uint16_t h30_expected_len = 0;

// 根据协议文档2.1.3节的校验和计算方法
typedef struct {
    uint8_t ck1;
    uint8_t ck2;
} h30_checksum_t;

static h30_checksum_t h30_calculate_checksum(uint8_t *data, uint16_t len) {
    h30_checksum_t checksum = {0, 0};
    
    for(uint16_t i = 0; i < len; i++) {
        checksum.ck1 += data[i];
        checksum.ck2 += checksum.ck1;
    }
    
    return checksum;
}
// 配置H30模块
// 配置H30模块
void h30_configure(void)
{
    uint8_t ack[64];
    uint16_t ack_len = 0;
    
    // 1. 首先查询当前输出内容
    uint8_t query_output_cmd[] = {0x59, 0x53, 0x04, 0x00, 0x00, 0x3B, 0x6E};
    HAL_UART_Transmit(&huart1, query_output_cmd, sizeof(query_output_cmd), 100);
    HAL_Delay(50);
    
    // 2. 配置输出内容：启用欧拉角(0x40)等必要数据
    // 数据域格式：2字节位图，按协议文档第11页定义
    // Bit10: IMU温度, Bit11: 组合导航状态, 确保欧拉角对应的位被设置
    uint8_t output_cmd[] = {0x59, 0x53, 0x04, 0x02, 0x00, 0x00, 0x08, 0x00, 0x00}; // 临时值，需要计算校验和
    
    // 计算校验和（从数据类0x04开始到数据域结束）
    uint8_t check_data[] = {0x04, 0x02, 0x00, 0x00, 0x08}; // 数据类+操作符/长度+数据域
    h30_checksum_t checksum = h30_calculate_checksum(check_data, sizeof(check_data));
    output_cmd[7] = checksum.ck1;
    output_cmd[8] = checksum.ck2;
    
    HAL_UART_Transmit(&huart1, output_cmd, sizeof(output_cmd), 100);
    HAL_Delay(50);
    
    // 3. 配置波特率460800
    uint8_t baud_cmd[] = {0x59, 0x53, 0x02, 0x04, 0x00, 0x04, 0x10, 0x2A};
    HAL_UART_Transmit(&huart1, baud_cmd, sizeof(baud_cmd), 100);
    HAL_Delay(100); // 波特率改变后需要更长的延时
    
    // 4. 配置输出频率100Hz
    uint8_t freq_cmd[] = {0x59, 0x53, 0x03, 0x0A, 0x00, 0x08, 0x15, 0x32};
    HAL_UART_Transmit(&huart1, freq_cmd, sizeof(freq_cmd), 100);
    HAL_Delay(50);
    
    // 5. 配置为VRU模式
    uint8_t mode_cmd[] = {0x59, 0x53, 0x4D, 0x12, 0x00, 0x02, 0x02, 0x63, 0xCF};
    HAL_UART_Transmit(&huart1, mode_cmd, sizeof(mode_cmd), 100);
    HAL_Delay(50);
}

// 字节转浮点数（小端序）
float h30_bytes_to_float(uint8_t *bytes)
{
    int32_t temp = ((int32_t)bytes[3] << 24) | 
                   ((int32_t)bytes[2] << 16) | 
                   ((int32_t)bytes[1] << 8) | 
                   bytes[0];
    return *(float*)&temp * 0.000001f;  // 微度转度
}

// 修改解析函数，添加校验验证
void h30_parse_data(uint8_t *data, uint16_t len) {
    if (len < 7) return;
    
    // 解析帧头
    if (data[0] != 0x59 || data[1] != 0x53) return;
    
    uint16_t tid = (data[3] << 8) | data[2]; // 小端模式
    uint8_t payload_len = data[4];
    
    // 验证帧长度
    uint16_t total_len = payload_len + 7; // 帧头2 + 帧编号2 + 长度1 + 数据 + 校验2
    if (len < total_len) return;
    
    // 验证校验和（根据协议文档第9页）
    // 校验范围：从TID开始（偏移2）到Message结束（偏移4+payload_len）
    uint16_t check_len = 2 + 1 + payload_len; // TID(2) + LEN(1) + 数据域
    h30_checksum_t checksum = h30_calculate_checksum(&data[2], check_len);
    
    uint8_t received_ck1 = data[total_len - 2];
    uint8_t received_ck2 = data[total_len - 1];
    
    if (checksum.ck1 != received_ck1 || checksum.ck2 != received_ck2) {
        // 校验失败
        return;
    }
    
    uint8_t *payload_ptr = &data[5];
    uint16_t pos = 0;
    
    while (pos < payload_len) {
        if (pos + 2 > payload_len) break;
        
        uint8_t data_id = payload_ptr[pos];
        uint8_t data_len = payload_ptr[pos + 1];
        
        if (pos + 2 + data_len > payload_len) break;
        
        uint8_t *packet_data = payload_ptr + pos + 2;
        
        if (data_id == 0x40 && data_len >= 12) {
            // 欧拉角数据包
            // 每个角度4字节，小端模式，有符号int32
            int32_t pitch_raw = (int32_t)((packet_data[3] << 24) | (packet_data[2] << 16) | 
                                          (packet_data[1] << 8) | packet_data[0]);
            int32_t roll_raw = (int32_t)((packet_data[7] << 24) | (packet_data[6] << 16) | 
                                         (packet_data[5] << 8) | packet_data[4]);
            int32_t yaw_raw = (int32_t)((packet_data[11] << 24) | (packet_data[10] << 16) | 
                                        (packet_data[9] << 8) | packet_data[8]);
            preangle.pit = Angle_Measure.pit;
					  preangle.yaw = Angle_Measure.yaw;
					  preangle.rol = Angle_Measure.rol;
            // 转换为浮点数度（乘以0.000001）
            float pitch_raw_deg = pitch_raw * 0.000001f;
            float roll_raw_deg = roll_raw * 0.000001f;
            float yaw_raw_deg = yaw_raw * 0.000001f;
            
            // 如果是第一次有效数据，记录为偏移量
            if (!imu_offset_calibrated) {
                imu_offset.pit = pitch_raw_deg;
                imu_offset.rol = roll_raw_deg;
                imu_offset.yaw = yaw_raw_deg;
                imu_offset_calibrated = 1;  // 标记已校准
            }
            
            // 减去偏移量，得到校正后的角度
            Angle_Measure.pit = pitch_raw_deg - imu_offset.pit;
            Angle_Measure.rol = roll_raw_deg - imu_offset.rol;
            Angle_Measure.yaw = yaw_raw_deg - imu_offset.yaw;
            
            imu_data_ready = 1;
            break;
        }
        
        pos += 2 + data_len;
    }
}

// 修改状态机，重置逻辑更清晰
uint8_t h30_data_callback(uint8_t byte) {
    switch (h30_state) {
        case H30_STATE_IDLE:
            if (byte == 0x59) {
                h30_state = H30_STATE_HEADER1;
                h30_rx_index = 0;
                h30_rx_buffer[h30_rx_index++] = byte;
            }
            break;
            
        case H30_STATE_HEADER1:
            if (byte == 0x53) {
                h30_state = H30_STATE_HEADER2;
                h30_rx_buffer[h30_rx_index++] = byte;
            } else {
                h30_state = H30_STATE_IDLE;
            }
            break;
            
        case H30_STATE_HEADER2:
            h30_rx_buffer[h30_rx_index++] = byte;
            
            if (h30_rx_index == 5) {
                h30_header_t *header = (h30_header_t*)h30_rx_buffer;
                uint8_t data_len = header->len;
                
                // 验证数据长度有效
                if (data_len <= 255) {
                    h30_expected_len = data_len + 7;
                    h30_state = H30_STATE_RECEIVING;
                } else {
                    // 长度无效，重置状态机
                    h30_state = H30_STATE_IDLE;
                }
            }
            break;
            
        case H30_STATE_RECEIVING:
            h30_rx_buffer[h30_rx_index++] = byte;
            
            // 检查是否接收完整帧
            if (h30_rx_index >= h30_expected_len) {
                // 解析完整帧
                h30_parse_data(h30_rx_buffer, h30_rx_index);
                
                // 重置状态机
                h30_state = H30_STATE_IDLE;
                h30_rx_index = 0;
                h30_expected_len = 0;
            } else if (h30_rx_index >= H30_MAX_FRAME_LEN) {
                // 缓冲区溢出，重置
                h30_state = H30_STATE_IDLE;
                h30_rx_index = 0;
                h30_expected_len = 0;
            }
            break;
    }
    
    return imu_data_ready;
}

// 重置偏移量校准（可选功能，用于重新校准）
void imu_reset_offset(void) {
    imu_offset_calibrated = 0;
    imu_offset.pit = 0.0f;
    imu_offset.rol = 0.0f;
    imu_offset.yaw = 0.0f;
}
