
#ifndef BDS_UART_H
#define BDS_UART_H

/*****************************************************************************
 *  Includes
 *****************************************************************************/

/*****************************************************************************
 *  Macro Definitions
 *****************************************************************************/
#define RET_OK 0
#define RET_NG -1
#define PIPE_IS_FULL            (1)
#define PIPE_IS_NOT_FULL        (0)
#define PIPE_IS_EMPTY           (1)
#define PIPE_IS_NOT_EMPTY       (0)
#define PIPE_IS_IN_ERR          (-1)
#define PIPE_STATUS_INIT        (0)
#define PIPE_STATUS_WRITTEN     (1)
#define PIPE_STATUS_READED      (2)
#define FRAME_MAX_LEN 32
#define UART_PIPE_SIZE          1500
#define PKG_MAX_LEN             150
#define BDS_DATA_LEN            256
#define PKG_COUNT               10
/*****************************************************************************
 *  Type Definitions
 *****************************************************************************/
///Pipe_Pkg store bds data
typedef struct pipe_pkg
{
    uint16_t status;//管道元素状态:0-初始态，1-写完成，2-读完成
    uint16_t packet_len;//valid data length
    uint8_t packet_buf[PKG_MAX_LEN];
}pipe_pkg_t;

typedef struct uart_pipe
{
    uint32_t pipe_init;
    uint32_t pipe_rd_cnt;
    uint32_t pipe_wr_cnt;
    pipe_pkg_t pipe_buf[UART_PIPE_SIZE];
}uart_pipe_t;
/*****************************************************************************
 *  External Global Variable Declarations
 *****************************************************************************/

/*****************************************************************************
 *  Function Prototype Declarations
 *****************************************************************************/

/*串口接收数据管道操作*/
int uart_pipe_create(void);
int uart_pipe_pop(void *ppipe_pkg, uint16_t *ppkg_len);
int uart_pipe_push(void *ppipe_pkg, uint16_t pkg_len);
int uart_pipe_is_empty(void);
int uart_pipe_is_full(void);
int uart_pipe_destory(void);
int uart_pipe_flush(void);


int bds_thread_init();
void bds_analysis(char *line);

#endif
