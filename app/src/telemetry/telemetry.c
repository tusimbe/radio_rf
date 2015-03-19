#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include <common/mavlink.h>
#include "telemetry.h"

mavlink_system_t mavlink_system = {20, 3, 0, 0 ,0 ,0};
mavlink_system_t target_system;

UART_HandleTypeDef huart2;
TELEMETRY_DATA telemetry;
osSemaphoreId telemetry_sema;
osThreadId telemetryTaskHandle;
static uint32_t packet_drops;
static bool mavlink_active = false;
static bool request_send = false;

uint8_t buf[MAVLINK_MAX_PACKET_LEN];
TELEMETRY_STREAM streams[] = 
{
    {MAV_DATA_STREAM_RAW_SENSORS,          2},
    {MAV_DATA_STREAM_EXTENDED_STATUS,      2},
    {MAV_DATA_STREAM_RC_CHANNELS,          5},
    {MAV_DATA_STREAM_POSITION,             2},
    {MAV_DATA_STREAM_EXTRA1,               5},
    {MAV_DATA_STREAM_EXTRA2,               2}
};

uint32_t streams_max = sizeof(streams) / sizeof(streams[0]);

void telemetry_uart2_init(void);
void telemetry_process_task(void const *argument);
void telemetry_mavlink_proc(uint8_t c);
void telemetry_data_request_read(void);
void USART2_IRQHandler(void);

int32_t telemetry_init(void)
{
    telemetry_uart2_init();
    
    osSemaphoreDef(TELEMETRY_SEM);
    telemetry_sema = osSemaphoreEmptyCreate(osSemaphore(TELEMETRY_SEM));
    if (NULL == telemetry_sema)
    {
        printf("[%s, L%d] create semaphore failed ret 0x%x.\r\n", 
            __FILE__, __LINE__, (unsigned int)telemetry_sema);
        return -1;
    }

    osThreadDef(telemetryTask, telemetry_process_task, osPriorityNormal, 0, 1024);
    telemetryTaskHandle = osThreadCreate(osThread(telemetryTask), NULL);
    if (NULL == telemetryTaskHandle)
    {
        printf("[%s, L%d] create thread failed.\r\n", 
            __FILE__, __LINE__);
        return -1;        
    }

    HAL_NVIC_SetPriority(USART2_IRQn, configLIBRARY_LOWEST_INTERRUPT_PRIORITY, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
    
    return 0;
}

/* USART2 init function */
void telemetry_uart2_init(void)
{

    huart2.Instance = USART2;
    huart2.Init.BaudRate = 57600;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart2);

    return;
}

void telemetry_mavlink_proc(uint8_t c)
{
    mavlink_message_t msg;
    mavlink_status_t  status;
    
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status))
    {
        switch (msg.msgid)
        {
            case MAVLINK_MSG_ID_HEARTBEAT:
                mavlink_active = true;
                target_system.sysid = msg.sysid;
                target_system.compid = msg.compid;
                (void)osSemaphoreReleaseFromISR(telemetry_sema);
                break;
            case MAVLINK_MSG_ID_RAW_IMU:
                telemetry.raw_imu.xgyro = mavlink_msg_raw_imu_get_xgyro(&msg);
                telemetry.raw_imu.ygyro = mavlink_msg_raw_imu_get_ygyro(&msg);
                telemetry.raw_imu.zgyro = mavlink_msg_raw_imu_get_zgyro(&msg);
                telemetry.raw_imu.xacc  = mavlink_msg_raw_imu_get_xacc(&msg);
                telemetry.raw_imu.yacc  = mavlink_msg_raw_imu_get_yacc(&msg);
                telemetry.raw_imu.zacc  = mavlink_msg_raw_imu_get_zacc(&msg);
                telemetry.raw_imu.xmag  = mavlink_msg_raw_imu_get_xmag(&msg);
                telemetry.raw_imu.ymag  = mavlink_msg_raw_imu_get_ymag(&msg);
                telemetry.raw_imu.zmag  = mavlink_msg_raw_imu_get_zmag(&msg);
                break;
            case MAVLINK_MSG_ID_SYS_STATUS:
                telemetry.sys_status.voltage_battery = 
                    mavlink_msg_sys_status_get_voltage_battery(&msg);
                telemetry.sys_status.current_battery = 
                    mavlink_msg_sys_status_get_current_battery(&msg);
                telemetry.sys_status.battery_remaining =
                    mavlink_msg_sys_status_get_battery_remaining(&msg);
                break;
            case MAVLINK_MSG_ID_ATTITUDE:
                telemetry.attitude.roll  = mavlink_msg_attitude_get_roll(&msg);
                telemetry.attitude.pitch = mavlink_msg_attitude_get_pitch(&msg);
                telemetry.attitude.yaw   = mavlink_msg_attitude_get_yaw(&msg);
                break;
            case MAVLINK_MSG_ID_GPS_RAW_INT:
                telemetry.gps_raw.lat = mavlink_msg_gps_raw_int_get_lat(&msg);
                telemetry.gps_raw.lon = mavlink_msg_gps_raw_int_get_lon(&msg);
                telemetry.gps_raw.alt = mavlink_msg_gps_raw_int_get_alt(&msg);
                telemetry.gps_raw.fix_type = mavlink_msg_gps_raw_int_get_fix_type(&msg);
                telemetry.gps_raw.satellites_visible = mavlink_msg_gps_raw_int_get_satellites_visible(&msg);
            default:
                break;
                
        }
    }

    packet_drops += status.packet_rx_drop_count;

    return;
}

void USART2_IRQHandler(void)
{
    uint8_t c;
    if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE) != RESET)
    {
        c = huart2.Instance->DR;
        //printf("0x%x \r\n", c);
        telemetry_mavlink_proc(c);
    }
}

void telemetry_data_request_read(void)
{
    mavlink_message_t msg;
    uint16_t len;
    uint32_t i;

    for (i = 0; i < streams_max; i++)
    {
        mavlink_msg_request_data_stream_pack(mavlink_system.sysid, 
            mavlink_system.compid, &msg, target_system.sysid, target_system.compid, 
            streams[i].stream_id, streams[i].freq, 1);
            
        len = mavlink_msg_to_send_buffer(buf, &msg);

        HAL_UART_Transmit(&huart2, buf, len, 5000);
    }

    return;
}


uint8_t telemetry_push_volt_cur
(
    void *buf
)
{
    uint8_t *ptr = buf;
    uint8_t len;
    
    *(uint16_t*)ptr = telemetry.sys_status.voltage_battery;
    ptr += sizeof(uint16_t);
    len = sizeof(uint16_t);
    
    *(int16_t*)ptr = telemetry.sys_status.current_battery;
    ptr += sizeof(int16_t);
    len += sizeof(int16_t);
    
    *(uint16_t*)ptr = telemetry.sys_status.battery_remaining;
    len += sizeof(uint16_t);

    return len;
}

uint8_t telemetry_push_attitude
(
    void *buf
)
{
    uint8_t *ptr = buf;
    uint8_t len;
    
    *(float*)ptr = telemetry.attitude.roll;
    ptr += sizeof(float);
    len = sizeof(float);
    
    *(float*)ptr = telemetry.attitude.pitch;
    ptr += sizeof(float);
    len += sizeof(float);
    
    *(float*)ptr = telemetry.attitude.yaw;
    len += sizeof(float);

    return len;
}

void telemetry_process_task(void const *argument)
{
    uint32_t i;
    argument = argument;
    
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);

    for (;;)
    {
        /* waitting for data update notify */
        (void)osSemaphoreWait(telemetry_sema, osWaitForever);

        if (mavlink_active && !request_send)
        {
            for (i = 0; i < 3; i++)
            {
                telemetry_data_request_read();
                request_send = true;
            }
        }


        printf("[raw imu]xg:%d, yg:%d, zg:%d\r\n", telemetry.raw_imu.xgyro, telemetry.raw_imu.ygyro, 
        telemetry.raw_imu.zgyro);
        printf("[attitude]roll:%f pitch:%f yaw:%f\r\n", ToDeg(telemetry.attitude.roll), 
        ToDeg(telemetry.attitude.pitch), ToDeg(telemetry.attitude.yaw));
    }
}

