/*
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-08-15     misonyo      first implementation.
 * 2024-09-08     silentowl    for bds modules modify
 *
 */
/*****************************************************************************
 *  Includes
 *****************************************************************************/
#include <rtthread.h>
#include <stdio.h>
#include <unistd.h>
#include <ipc/dataqueue.h>
#include <ipc/completion.h>
#include <drivers/serial.h>
#include "bds_uart.h"
#include "minmea.h"

/*****************************************************************************
 *  Internal Macro Definitions
 *****************************************************************************/
#define LOG_TAG "uart"
#define LOG_LVL LOG_LVL_DBG
#include <ulog.h>

#define BDS_UART_NAME           "uart1"
#define INDENT_SPACES           "  "
#define USE_INTERRUPT
#define TEMP_BUF_LEN           (BDS_DATA_LEN*2)
/*****************************************************************************
 *  Internal Type Definitions
 *****************************************************************************/

/*****************************************************************************
 *  Internal Structure Definitions
 *****************************************************************************/

/*****************************************************************************
 *  Internal Function Declarations
 *****************************************************************************/
void show_bytes(const char *start, int len);
static rt_err_t uart_input(rt_device_t dev, rt_size_t size);
static void serial_thread_entry(void *parameter);
static void bds_read_thread_entry(void *parameter);
/*****************************************************************************
 *  Internal Variable Definitions
 *****************************************************************************/
/* 用于接收消息的信号量 */
static struct rt_semaphore rx_sem;
/*串口设备句柄*/
static rt_device_t serial;
/*串口接收缓冲区*/
static char uart_recv[BDS_DATA_LEN];
/*bds数据存储管道*/
static uart_pipe_t bds_uart_pipe;
static uint8_t count;
/*****************************************************************************
 *  External Global Variable Declarations
 *****************************************************************************/

/*****************************************************************************
 *  Global Function Definitions
 *****************************************************************************/


int uart_pipe_create(void)
{
    memset((void *)&bds_uart_pipe, 0x00, sizeof(bds_uart_pipe));
    bds_uart_pipe.pipe_init = 0xff;
    rt_kprintf("pipe init: %d\n", bds_uart_pipe.pipe_init);
    return RT_EOK;
}

int uart_pipe_pop(void *ppipe_pkg, uint16_t *ppkg_len)
{
    if(ppipe_pkg == NULL)
    {
        return RT_ERROR;
    }
    if(bds_uart_pipe.pipe_init <= 0)
    {
        return RT_ERROR+1;
    }
    if(bds_uart_pipe.pipe_rd_cnt >= UART_PIPE_SIZE)
    {
        return RT_ERROR+2;
    }
    if(bds_uart_pipe.pipe_buf[bds_uart_pipe.pipe_rd_cnt].status != PIPE_STATUS_WRITTEN)
    {
        return RT_ERROR+3;
    }
    memcpy(ppipe_pkg, (void *)&(bds_uart_pipe.pipe_buf[bds_uart_pipe.pipe_rd_cnt].packet_buf),
            bds_uart_pipe.pipe_buf[bds_uart_pipe.pipe_rd_cnt].packet_len);
    *ppkg_len = bds_uart_pipe.pipe_buf[bds_uart_pipe.pipe_rd_cnt].packet_len;
    bds_uart_pipe.pipe_buf[bds_uart_pipe.pipe_rd_cnt].status = PIPE_STATUS_READED;
    bds_uart_pipe.pipe_rd_cnt = bds_uart_pipe.pipe_rd_cnt + 1;
    if(bds_uart_pipe.pipe_rd_cnt >= UART_PIPE_SIZE)
    {
        bds_uart_pipe.pipe_rd_cnt = (bds_uart_pipe.pipe_rd_cnt) % UART_PIPE_SIZE;
    }
    return RT_EOK;
}

int uart_pipe_push(void *ppipe_pkg, uint16_t pkg_len)
{
    if(ppipe_pkg == NULL)
    {
        return RT_ERROR;
    }
    if(bds_uart_pipe.pipe_init <= 0)
    {
        return RT_ERROR;
    }
    if(bds_uart_pipe.pipe_wr_cnt >= UART_PIPE_SIZE)
    {
        return RT_ERROR;
    }
    if(bds_uart_pipe.pipe_buf[bds_uart_pipe.pipe_wr_cnt].status == PIPE_STATUS_WRITTEN)
    {
        return RT_ERROR;
    }
    memcpy((void *)&(bds_uart_pipe.pipe_buf[bds_uart_pipe.pipe_wr_cnt].packet_buf), ppipe_pkg, pkg_len);
    bds_uart_pipe.pipe_buf[bds_uart_pipe.pipe_wr_cnt].packet_len = pkg_len;
    bds_uart_pipe.pipe_buf[bds_uart_pipe.pipe_wr_cnt].status = PIPE_STATUS_WRITTEN;
    bds_uart_pipe.pipe_wr_cnt = bds_uart_pipe.pipe_wr_cnt + 1;
    if(bds_uart_pipe.pipe_wr_cnt >= UART_PIPE_SIZE)
    {
        bds_uart_pipe.pipe_wr_cnt = bds_uart_pipe.pipe_wr_cnt % UART_PIPE_SIZE;
    }
    return RT_EOK;
}

int uart_pipe_is_empty(void)
{
    if(bds_uart_pipe.pipe_init <= 0)
    {
        return RT_ERROR;
    }
    if(bds_uart_pipe.pipe_rd_cnt >= UART_PIPE_SIZE)
    {
        return RT_ERROR;
    }
    if(bds_uart_pipe.pipe_buf[bds_uart_pipe.pipe_rd_cnt].status != PIPE_STATUS_WRITTEN)
    {
        return PIPE_IS_EMPTY;
    }
    return PIPE_IS_NOT_EMPTY;
}

int uart_pipe_is_full(void)
{
    if(bds_uart_pipe.pipe_init <= 0)
    {
        return RT_ERROR;
    }
    if(bds_uart_pipe.pipe_wr_cnt >= UART_PIPE_SIZE)
    {
        return RT_ERROR;
    }
    if(bds_uart_pipe.pipe_buf[bds_uart_pipe.pipe_wr_cnt].status == PIPE_STATUS_WRITTEN)
    {
        if(count > PKG_COUNT)
        {
            //GetCurPipeTime();
            rt_kprintf("pipe read status: PIPE_IS_FULL\n");
            count = 0;
        }
        return PIPE_IS_FULL;
    }
    return PIPE_IS_NOT_FULL;
}

int uart_pipe_destory(void)
{
    bds_uart_pipe.pipe_init = 0;
    memset(bds_uart_pipe.pipe_buf, 0, sizeof(bds_uart_pipe));
    return RT_EOK;
}

/*清空pipe*/
int uart_pipe_flush(void)
{
    if(bds_uart_pipe.pipe_init <= 0)
    {
        return RT_ERROR;
    }
    memset(bds_uart_pipe.pipe_buf, 0x00, sizeof(pipe_pkg_t)*UART_PIPE_SIZE);
    bds_uart_pipe.pipe_rd_cnt = 0;
    bds_uart_pipe.pipe_wr_cnt = 0;
    return RT_EOK;
}

#ifdef USE_INTERRUPT
/* 接收数据回调函数*/
static rt_err_t uart_input(rt_device_t dev, rt_size_t size)
{
    rt_sem_release(&rx_sem);
    return RT_EOK;
}

void show_bytes(const char *start, int len)
{
    int i;
    for (i = 0; i < len; i++)
    {
        printf(" %.2x", start[i]);    //line:data:show_bytes_printf
    }
    printf("\n");
}

void find_bds_msg(char *data_buf, uint16_t data_len, uint16_t *remain_len)
{
    uint16_t i = 0;
    uint16_t start_pos = 0, end_pos=0;
    bool msg_start_flag = false;
    if ((data_buf == NULL) || (remain_len == NULL)) {
        return;
    }
    while(i < TEMP_BUF_LEN)
    {
       if (data_buf[i] == '\0') {
           break;
       }
       if(data_buf[i]=='$')
       {
           start_pos = i++;
           msg_start_flag = true;
       }
       else if (data_buf[i] == '\r' && data_buf[i+1] == '\n')
       {
           if (msg_start_flag == true)
           {
               i+=2;
               end_pos = i; //end_pos在\n之后
               //rt_kprintf("startpos=%d, endpos=%d\n", start_pos, end_pos);
               uart_pipe_push((void *)data_buf+start_pos, end_pos-start_pos);
               if(uart_pipe_is_full() == PIPE_IS_FULL)
               {
                   rt_kprintf("[bds_uart.c]uart_pipe_is_full\n");
               }
              // rt_kprintf("uart_pipe_push:%s, len = %d\n", tmp_buf, strlen(tmp_buf));
               msg_start_flag = false;
            }
            else
            {
                i++;
                rt_kprintf("[bds_uart.c]do not have start identifier\n");//to do
            }
       }
       else {
           i++;
       }
    }
    if ((end_pos != 0) && (msg_start_flag == true) && (end_pos == start_pos)) {
        *remain_len = i-start_pos;
        memcpy(data_buf, data_buf+start_pos, *remain_len);
        memset(data_buf+*remain_len, 0x00, data_len-*remain_len);
        //rt_kprintf("[bds_uart.c]remain data_buf=%s, remain_len = %d\n", data_buf, *remain_len);//to do
    }
    else {
        *remain_len = 0;
        memset(data_buf, 0x00, data_len);
    }
}

static void serial_thread_entry(void *parameter)
{
    char local_buf[TEMP_BUF_LEN] = "";
    uint16_t remain_len = 0;
    while (1)
    {
        while (rt_device_read(serial, -1, uart_recv, BDS_DATA_LEN) != 1)
        {
            rt_sem_take(&rx_sem, RT_WAITING_FOREVER);
        }
        strncpy(local_buf+remain_len, uart_recv, BDS_DATA_LEN);
        //rt_kprintf("[bds_uart.c]uart recv local_buf:%s, len = %d\n", local_buf, BDS_DATA_LEN+remain_len);
        find_bds_msg(local_buf, TEMP_BUF_LEN, &remain_len);
        rt_thread_mdelay(50);
    }
}

static void bds_read_thread_entry(void *parameter)
{
    char loc_bds_read_buf[PKG_MAX_LEN] = "";
    uint16_t len = 0;
    int ret = 0;
    while (1)
    {
        ret = uart_pipe_pop((void *)loc_bds_read_buf, &len);
        if (RT_EOK == ret) {
            rt_kprintf("[bds_uart.c]uart_pipe_pop:%s, len=%d, ret=%d\n", loc_bds_read_buf, len, ret);
            bds_analysis(loc_bds_read_buf);
            memset(loc_bds_read_buf, 0x00, PKG_MAX_LEN);
        }
        rt_thread_mdelay(5);
        len = 0;
    }
}
#endif

#ifndef USE_INTERRUPT
/* 串 口 接 收 消 息 结 构*/
struct rx_msg
{
    rt_device_t dev;
    rt_size_t size;
};

/* 消 息 队 列 控 制 块 */
static struct rt_messagequeue rx_mq;
/* 接 收 数 据 回 调 函 数 */
static rt_err_t uart_input(rt_device_t dev, rt_size_t size)
{
    struct rx_msg msg;
    rt_err_t result;
    msg.dev = dev;
    msg.size = size;
    result = rt_mq_send(&rx_mq, &msg, sizeof(msg));
    if ( result == -RT_EFULL)
    {
        /* 消 息 队 列 满 */
        rt_kprintf("message queue full！\n");
    }
    return result;
}

static void serial_thread_entry(void *parameter)
{
    struct rx_msg msg;
    rt_err_t result;
    rt_uint32_t rx_length;
    static char rx_buffer[128 + 1];
    while (1)
    {
        rt_memset(&msg, 0, sizeof(msg));
        /* 从 消 息 队 列 中 读 取 消 息*/
        result = rt_mq_recv(&rx_mq, &msg, sizeof(msg), RT_WAITING_FOREVER);
        if (result == RT_EOK)
        {
            /* 从 串 口 读 取 数 据*/
            rx_length = rt_device_read(msg.dev, 0, rx_buffer, msg.size);
            rx_buffer[rx_length] = '\0';
            /* 打 印 数 据 */
            rt_kprintf("%s\n",rx_buffer);
        }
    }
}
#endif

void bds_analysis(char *line)
{
    switch (minmea_sentence_id(line, false)) {
      case MINMEA_SENTENCE_RMC: {
          struct minmea_sentence_rmc frame;
          if (minmea_parse_rmc(&frame, line)) {
              rt_kprintf(INDENT_SPACES "$xxRMC: raw coordinates and speed: (%d/%d,%d/%d) %d/%d\n",
                      frame.latitude.value, frame.latitude.scale,
                      frame.longitude.value, frame.longitude.scale,
                      frame.speed.value, frame.speed.scale);
              rt_kprintf(INDENT_SPACES "$xxRMC fixed-point coordinates and speed scaled to three decimal places: (%d,%d) %d\n",
                      minmea_rescale(&frame.latitude, 1000),
                      minmea_rescale(&frame.longitude, 1000),
                      minmea_rescale(&frame.speed, 1000));
              rt_kprintf(INDENT_SPACES "$xxRMC floating point degree coordinates and speed: (%f,%f) %f\n",
                      minmea_tocoord(&frame.latitude),
                      minmea_tocoord(&frame.longitude),
                      minmea_tofloat(&frame.speed));
          }
          else {
              rt_kprintf(INDENT_SPACES "$xxRMC sentence is not parsed\n");
          }
      } break;

      case MINMEA_SENTENCE_GGA: {
          struct minmea_sentence_gga frame;
          if (minmea_parse_gga(&frame, line)) {
              rt_kprintf(INDENT_SPACES "$xxGGA: fix quality: %d\n", frame.fix_quality);
          }
          else {
              rt_kprintf(INDENT_SPACES "$xxGGA sentence is not parsed\n");
          }
      } break;

      case MINMEA_SENTENCE_GST: {
          struct minmea_sentence_gst frame;
          if (minmea_parse_gst(&frame, line)) {
              rt_kprintf(INDENT_SPACES "$xxGST: raw latitude,longitude and altitude error deviation: (%d/%d,%d/%d,%d/%d)\n",
                      frame.latitude_error_deviation.value, frame.latitude_error_deviation.scale,
                      frame.longitude_error_deviation.value, frame.longitude_error_deviation.scale,
                      frame.altitude_error_deviation.value, frame.altitude_error_deviation.scale);
              rt_kprintf(INDENT_SPACES "$xxGST fixed point latitude,longitude and altitude error deviation"
                     " scaled to one decimal place: (%d,%d,%d)\n",
                      minmea_rescale(&frame.latitude_error_deviation, 10),
                      minmea_rescale(&frame.longitude_error_deviation, 10),
                      minmea_rescale(&frame.altitude_error_deviation, 10));
              rt_kprintf(INDENT_SPACES "$xxGST floating point degree latitude, longitude and altitude error deviation: (%f,%f,%f)",
                      minmea_tofloat(&frame.latitude_error_deviation),
                      minmea_tofloat(&frame.longitude_error_deviation),
                      minmea_tofloat(&frame.altitude_error_deviation));
          }
          else {
              rt_kprintf(INDENT_SPACES "$xxGST sentence is not parsed\n");
          }
      } break;

      case MINMEA_SENTENCE_GSV: {
          struct minmea_sentence_gsv frame;
          if (minmea_parse_gsv(&frame, line)) {
              rt_kprintf(INDENT_SPACES "$xxGSV: message %d of %d\n", frame.msg_nr, frame.total_msgs);
              rt_kprintf(INDENT_SPACES "$xxGSV: satellites in view: %d\n", frame.total_sats);
              for (int i = 0; i < 4; i++)
                  rt_kprintf(INDENT_SPACES "$xxGSV: sat nr %d, elevation: %d, azimuth: %d, snr: %d dbm\n",
                      frame.sats[i].nr,
                      frame.sats[i].elevation,
                      frame.sats[i].azimuth,
                      frame.sats[i].snr);
          }
          else {
              rt_kprintf(INDENT_SPACES "$xxGSV sentence is not parsed\n");
          }
      } break;

      case MINMEA_SENTENCE_VTG: {
         struct minmea_sentence_vtg frame;
         if (minmea_parse_vtg(&frame, line)) {
             rt_kprintf(INDENT_SPACES "$xxVTG: true track degrees = %f\n",
                     minmea_tofloat(&frame.true_track_degrees));
             rt_kprintf(INDENT_SPACES "        magnetic track degrees = %f\n",
                     minmea_tofloat(&frame.magnetic_track_degrees));
             rt_kprintf(INDENT_SPACES "        speed knots = %f\n",
                      minmea_tofloat(&frame.speed_knots));
             rt_kprintf(INDENT_SPACES "        speed kph = %f\n",
                      minmea_tofloat(&frame.speed_kph));
         }
         else {
             rt_kprintf(INDENT_SPACES "$xxVTG sentence is not parsed\n");
         }
      } break;

      case MINMEA_SENTENCE_ZDA: {
          struct minmea_sentence_zda frame;
          if (minmea_parse_zda(&frame, line)) {
              rt_kprintf(INDENT_SPACES "$xxZDA: %d:%d:%d %02d.%02d.%d UTC%+03d:%02d\n",
                     frame.time.hours,
                     frame.time.minutes,
                     frame.time.seconds,
                     frame.date.day,
                     frame.date.month,
                     frame.date.year,
                     frame.hour_offset,
                     frame.minute_offset);
          }
          else {
              rt_kprintf(INDENT_SPACES "$xxZDA sentence is not parsed\n");
          }
      } break;
      case MINMEA_SENTENCE_GSA : {
          rt_kprintf(INDENT_SPACES "$xxGSA sentence will be added soon\n");
      }break;
      case MINMEA_INVALID: {
          rt_kprintf(INDENT_SPACES "$xxxxx sentence is not valid\n");
      } break;

      default: {
          rt_kprintf(INDENT_SPACES "$xxxxx sentence is not parsed\n");
      } break;
    }
}

/*北斗定位数据采集线程入口*/
int bds_thread_init()
{
    rt_err_t ret = RT_EOK;
    char uart_name[RT_NAME_MAX];
    rt_strncpy(uart_name, BDS_UART_NAME, RT_NAME_MAX);
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
    serial = rt_device_find(uart_name);
    config.baud_rate = BAUD_RATE_115200; //修改波特率为115200
    config.data_bits = DATA_BITS_8; //数据位8
    config.stop_bits = STOP_BITS_1; //停止位1
    config.bufsz = BDS_DATA_LEN; //修改缓冲区buff size 为128
    config.parity = PARITY_NONE; //无奇偶校验位
    rt_device_control(serial, RT_DEVICE_CTRL_CONFIG, &config);
    if (!serial)
    {
        rt_kprintf("[bds_uart.c]find %s failed!\n", uart_name);
        return RT_ERROR;
    }
#ifdef USE_INTERRUPT
    uart_pipe_create();
    /*初始化信号量*/
    rt_sem_init(&rx_sem, "rx_sem", 0, RT_IPC_FLAG_FIFO);
    /*以读写及中断接收方式打开串口设备*/
    rt_device_open(serial, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX);
    /*设置接收回调函数*/
    rt_device_set_rx_indicate(serial, uart_input);
    /* 创建 serial 线程*/
    rt_thread_t uart_thread = rt_thread_create("serial", serial_thread_entry, RT_NULL, 2048, 25, 10);
    /* 创建成功则启动线程*/
    if (uart_thread != RT_NULL)
    {
        rt_thread_startup(uart_thread);
    }
    else
    {
        ret = RT_ERROR;
    }
    rt_thread_t bds_read_thread = rt_thread_create("bds_read", bds_read_thread_entry, RT_NULL, 2048, 24, 10);
    /* 创建成功则启动线程*/
    if (bds_read_thread != RT_NULL)
    {
        rt_thread_startup(bds_read_thread);
    }
    else
    {
        ret = RT_ERROR;
    }

#endif
#ifndef USE_INTERRUPT
    static char msg_pool[256];
    /* 初 始 化 消 息 队 列 */
    rt_mq_init(&rx_mq, "rx_mq",
                msg_pool, /* 存 放 消 息 的 缓 冲 区 */
                sizeof(struct rx_msg), /* 一 条 消 息 的 最 大 长 度 */
                sizeof(msg_pool), /* 存 放 消 息 的 缓 冲 区 大 小 */
                RT_IPC_FLAG_FIFO); /* 如 果 有 多 个 线 程 等 待， 按 照 先 来 先 得 到 的 方 法分 配 消 息 */
    /* 以 DMA 接 收 及 轮 询 发 送 方 式 打 开 串 口 设 备 */
    rt_device_open(serial, RT_DEVICE_FLAG_DMA_RX);
    /* 设 置 接 收 回 调 函 数 */
    rt_device_set_rx_indicate(serial, uart_input);
    /* 创 建 serial 线 程 */
    rt_thread_t thread = rt_thread_create("serial", serial_thread_entry, RT_NULL,
            2048, 25, 10);
    /* 创 建 成 功 则 启 动 线 程 */
    if (thread != RT_NULL)
    {
        rt_thread_startup(thread);
    }
    else
    {
        ret = RT_ERROR;
    }
#endif


    return ret;
}
/* 导出到 msh 命令列表中 */
MSH_CMD_EXPORT(bds_thread_init, beidou recv thread);
