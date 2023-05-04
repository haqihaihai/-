#include "board.h"
#include <math.h>

#define _SCB_BASE       (0xE000E010UL)
#define _SYSTICK_CTRL   (*(rt_uint32_t *)(_SCB_BASE + 0x0))
#define _SYSTICK_LOAD   (*(rt_uint32_t *)(_SCB_BASE + 0x4))
#define _SYSTICK_VAL    (*(rt_uint32_t *)(_SCB_BASE + 0x8))
#define _SYSTICK_CALIB  (*(rt_uint32_t *)(_SCB_BASE + 0xC))
#define _SYSTICK_PRI    (*(rt_uint8_t  *)(0xE000ED23UL))


static uint32_t _SysTick_Config(rt_uint32_t ticks)
{
    if ((ticks - 1) > 0xFFFFFF)
    {
        return 1;
    }
    
    _SYSTICK_LOAD = ticks - 1; 
    _SYSTICK_PRI = 0xFF;
    _SYSTICK_VAL  = 0;
    _SYSTICK_CTRL = 0x07;  
    
    return 0;
}
#include "BSP_All.h"
void rt_hw_board_init()
{
    /* enable interrupt */
    __set_PRIMASK(0);
    /* System clock initialization */
    SystemCoreClockUpdate();
    /* disable interrupt */
    __set_PRIMASK(1);

    _SysTick_Config(SystemCoreClock / RT_TICK_PER_SECOND);
    //rt_hw_systick_init();

    /* Heap initialization */
#if defined(RT_USING_HEAP)
    rt_system_heap_init((void *)HEAP_BEGIN, (void *)HEAP_END);
#endif

    /* Set the shell console output device */
#if defined(RT_USING_CONSOLE) && defined(RT_USING_DEVICE)
    rt_console_set_device(RT_CONSOLE_DEVICE_NAME);
#endif

    /* Board underlying hardware initialization */
#ifdef RT_USING_COMPONENTS_INIT
    rt_components_board_init();
#endif
//	rodicmd();
//	USART3_Config();
//	NVIC_Configuration();
	BSP_ALL();
}

void SysTick_Handler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    rt_tick_increase();

    /* leave interrupt */
    rt_interrupt_leave();
}


#if defined(RT_USING_FINSH)
#define rt_ringbuffer_space_len(rb) ((rb)->buffer_size - rt_ringbuffer_data_len(rb))

struct rt_ringbuffer
{
    rt_uint8_t *buffer_ptr;

    rt_uint16_t read_mirror : 1;
    rt_uint16_t read_index : 15;
    rt_uint16_t write_mirror : 1;
    rt_uint16_t write_index : 15;

    rt_int16_t buffer_size;
};

enum rt_ringbuffer_state
{
    RT_RINGBUFFER_EMPTY,
    RT_RINGBUFFER_FULL,
    /* half full is neither full nor empty */
    RT_RINGBUFFER_HALFFULL,
};

rt_inline enum rt_ringbuffer_state rt_ringbuffer_status(struct rt_ringbuffer *rb)
{
    if (rb->read_index == rb->write_index)
    {
        if (rb->read_mirror == rb->write_mirror)
            return RT_RINGBUFFER_EMPTY;
        else
            return RT_RINGBUFFER_FULL;
    }
    return RT_RINGBUFFER_HALFFULL;
}

/** 
 * get the size of data in rb 
 */
rt_size_t rt_ringbuffer_data_len(struct rt_ringbuffer *rb)
{
    switch (rt_ringbuffer_status(rb))
    {
    case RT_RINGBUFFER_EMPTY:
        return 0;
    case RT_RINGBUFFER_FULL:
        return rb->buffer_size;
    case RT_RINGBUFFER_HALFFULL:
    default:
        if (rb->write_index > rb->read_index)
            return rb->write_index - rb->read_index;
        else
            return rb->buffer_size - (rb->read_index - rb->write_index);
    };
}

void rt_ringbuffer_init(struct rt_ringbuffer *rb,
                        rt_uint8_t           *pool,
                        rt_int16_t            size)
{
    RT_ASSERT(rb != RT_NULL);
    RT_ASSERT(size > 0);

    /* initialize read and write index */
    rb->read_mirror = rb->read_index = 0;
    rb->write_mirror = rb->write_index = 0;

    /* set buffer pool and size */
    rb->buffer_ptr = pool;
    rb->buffer_size = RT_ALIGN_DOWN(size, RT_ALIGN_SIZE);
}

/**
 * put a character into ring buffer
 */
rt_size_t rt_ringbuffer_putchar(struct rt_ringbuffer *rb, const rt_uint8_t ch)
{
    RT_ASSERT(rb != RT_NULL);

    /* whether has enough space */
    if (!rt_ringbuffer_space_len(rb))
        return 0;

    rb->buffer_ptr[rb->write_index] = ch;

    /* flip mirror */
    if (rb->write_index == rb->buffer_size-1)
    {
        rb->write_mirror = ~rb->write_mirror;
        rb->write_index = 0;
    }
    else
    {
        rb->write_index++;
    }

    return 1;
}
/**
 * get a character from a ringbuffer
 */
rt_size_t rt_ringbuffer_getchar(struct rt_ringbuffer *rb, rt_uint8_t *ch)
{
    RT_ASSERT(rb != RT_NULL);

    /* ringbuffer is empty */
    if (!rt_ringbuffer_data_len(rb))
        return 0;

    /* put character */
    *ch = rb->buffer_ptr[rb->read_index];

    if (rb->read_index == rb->buffer_size-1)
    {
        rb->read_mirror = ~rb->read_mirror;
        rb->read_index = 0;
    }
    else
    {
        rb->read_index++;
    }

    return 1;
}

/* �ڶ����֣�finsh ��ֲ�ԽӲ��� */
#define UART_RX_BUF_LEN 16
rt_uint8_t uart_rx_buf[UART_RX_BUF_LEN] = {0};
struct rt_ringbuffer  uart_rxcb;         /* ����һ�� ringbuffer cb */
static struct rt_semaphore shell_rx_sem; /* ����һ����̬�ź��� */

/**
  * @brief  USART1 GPIO ����,����ģʽ���á�115200 8-N-1
  * @param  None
  * @retval None
  */
void USART3_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
	/* ��ʼ�����ڽ��� ringbuffer  */
  rt_ringbuffer_init(&uart_rxcb, uart_rx_buf, UART_RX_BUF_LEN);

  /* ��ʼ�����ڽ������ݵ��ź��� */
  rt_sem_init(&(shell_rx_sem), "shell_rx", 0, 0);
	
	/* config USART1 clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOD,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);//ʹ��USART3ʱ��
	//����3��Ӧ���Ÿ���ӳ��
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_USART3);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_USART3);
	
	/* USART3 GPIO config */
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_8; //GPIOD9��GPIOD10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOD,&GPIO_InitStructure); //��ʼ��PD9��PD10
	
	USART_DeInit(USART3);
	/* USART3 mode config */
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART3, &USART_InitStructure);
	
	/* ʹ�ܴ���3�����ж� */
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	
	USART_Cmd(USART3, ENABLE);
}

/**
  * @brief  ����USART3�����ж�
  * @param  None
  * @retval None
  */
void NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure; 
	/* Configure the NVIC Preemption Priority Bits */  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	/* Enable the USARTy Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;	 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

/*���һ���ַ���ϵͳ���������������ɸ��� */
void rt_hw_console_output(const char *str)
{
	rt_size_t  i = 0, size = 0;
  char a = '\r';
	
	/*�����־λ*/
	USART_ClearFlag(USART3,USART_FLAG_TC);
	
	size = rt_strlen(str);
	for (i = 0; i < size; i++)
	{
		if (*(str + i) == '\n')
		{
			/* ����һ���ֽ����ݵ�USART1 */
			USART_SendData(USART3, (uint8_t) a);
			/* �ȴ�������� */
			while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);			
		}
		/* ����һ���ֽ����ݵ�USART1 */
		USART_SendData(USART3, (uint8_t) *(str+i));
		/* �ȴ�������� */		
		while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);	
	}
}

/* ��ֲ FinSH��ʵ�������н���, ��Ҫ��� FinSH Դ�룬Ȼ���ٶԽ� rt_hw_console_getchar */
/* �жϷ�ʽ */
char rt_hw_console_getchar(void)
{
    char ch = 0;

    /* �� ringbuffer ���ó����� */
    while (rt_ringbuffer_getchar(&uart_rxcb, (rt_uint8_t *)&ch) != 1)
    {
        rt_sem_take(&shell_rx_sem, RT_WAITING_FOREVER);
    } 
    return ch;   
}

float Test_Set; //���ڵ����趨ֵ
float Test_P;   //���ڵ���Pֵ
float Test_I;		//���ڵ���Dֵ
float Test_D;		//���ڵ���Dֵ
void Automatic_Tuning_Parameter(uint8_t receive_data)		//�ô��ڵ��κ��� Э��($S+xxx.xxx P+xxx.xxx I+xxx.xxx D+xxx.xxx)
{																												//$S+000000. P+000.000 I+0.00000 D+000.000 
	static uint8_t Usart3Data[255];
	static uint8_t Usart3DataCnt = 0;
	if((receive_data >= 48) && (receive_data <= 57)) receive_data -= 48;	//���ڴ��ڷ���ֵΪASCII ����ת��Ϊ����
	Usart3Data[Usart3DataCnt++] = receive_data;
	if(Usart3Data[0] != '$')                       				//֡ͷ����������
	{
		Usart3DataCnt = 0;
		return;
	}
	if(Usart3DataCnt < 40){return;}
	else
	{
		if(Usart3Data[2] == '+')
		{
//			Test_Set = Usart3Data[3]*100+Usart3Data[4]*10+Usart3Data[5]+(float)Usart3Data[7]/10+(float)Usart3Data[8]/100+(float)Usart3Data[9]/1000;
			Test_Set = Usart3Data[3]*100000+Usart3Data[4]*10000+Usart3Data[5]*1000+Usart3Data[6]*100+Usart3Data[7]*10+Usart3Data[8];
		}else if(Usart3Data[2] == '-'){
//			Test_Set = -(Usart3Data[3]*100+Usart3Data[4]*10+Usart3Data[5]+(float)Usart3Data[7]/10+(float)Usart3Data[8]/100+(float)Usart3Data[9]/1000);
			Test_Set = -(Usart3Data[3]*100000+Usart3Data[4]*10000+Usart3Data[5]*1000+Usart3Data[6]*100+Usart3Data[7]*10+Usart3Data[8]);
		}
		if(Usart3Data[12] == '+')
		{
			Test_P = Usart3Data[13]*100+Usart3Data[14]*10+Usart3Data[15]+(float)Usart3Data[17]/10+(float)Usart3Data[18]/100+(float)Usart3Data[19]/1000;
		}else if(Usart3Data[12] == '-'){
			Test_P = -(Usart3Data[13]*100+Usart3Data[14]*10+Usart3Data[15]+(float)Usart3Data[17]/10+(float)Usart3Data[18]/100+(float)Usart3Data[19]/1000);
		}
		if(Usart3Data[22] == '+') //$S+000.000 P+000.000 I+000.000 D+000.000
		{													//										 I+0.00000
			Test_I = Usart3Data[23]+(float)Usart3Data[25]/10+(float)Usart3Data[26]/100+(float)Usart3Data[27]/1000+(float)Usart3Data[28]/10000+(float)Usart3Data[29]/100000;
		}else if(Usart3Data[22] == '-'){
			Test_I = -(Usart3Data[23]+(float)Usart3Data[25]/10+(float)Usart3Data[26]/100+(float)Usart3Data[27]/1000+(float)Usart3Data[28]/10000+(float)Usart3Data[29]/100000);
		}
		if(Usart3Data[32] == '+')
		{
			Test_D = Usart3Data[33]*100+Usart3Data[34]*10+Usart3Data[35]+(float)Usart3Data[37]/10+(float)Usart3Data[38]/100+(float)Usart3Data[39]/1000;
		}else if(Usart3Data[32] == '-'){
			Test_D = -(Usart3Data[33]*100+Usart3Data[34]*10+Usart3Data[35]+(float)Usart3Data[37]/10+(float)Usart3Data[38]/100+(float)Usart3Data[39]/1000);
		}
		
	}
	Usart3DataCnt = 0;
}

int flag1=0;
int flag2=0;
int nm1=0;
int nm2=0;
int nm3=0;
int nm4=0;
int x=90;
extern MOTOR_SEN_T motor_send_go8[2];

void kongzai_lijv(unsigned char rsg)
{
	if(flag1==2)
	{
		flag2++;
		if(rsg=='d'&&flag2==2)
		{
			nm1++;
			nm2++;
			nm3--;
			nm4--;
			x++;
			flag2=0;
			flag1=0;
		}
		else
		{
			flag2=0;
			flag1=0;
		}
	}
	if(flag1==1)
	{
		flag2++;
		if(rsg=='c'&&flag2==1)
		{
			flag1=2;
		}
		else
		{
			flag2=0;
			flag1=0;
		}
	}
	if(rsg=='a'&&flag1==0)
	{
		flag1=1;
	}
	if(rsg=='D')
	{
		nm1--;
		nm2--;
		nm3++;
		nm4++;
		x--;
	}
	if(rsg=='O')
	{
		nm1=0;
		nm2=0;
		nm3=0;
		nm4=0;
		x=0;
	}
}


/* �������֣��жϲ���*/
void USART3_IRQHandler(void)
{  
	int ch = -1;   
	unsigned char Rsg;
  /* enter interrupt */
	rt_interrupt_enter();          //���ж���һ��Ҫ������Ժ����������ж�
	
	if( (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) && (USART_GetFlagStatus(USART3, USART_FLAG_RXNE) != RESET) )
	{ 	
			while (1)      
			{
				ch = -1;
				if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
				{
					ch = USART_ReceiveData(USART3);
					Rsg=USART_ReceiveData(USART3);
					kongzai_lijv(Rsg);
				}
				if(ch ==-1)
				{
					break;
				}
				/* ��ȡ�����ݣ������ݴ��� ringbuffer */
				rt_ringbuffer_putchar(&uart_rxcb, ch);
			}
			rt_sem_release(&shell_rx_sem);
	} 
	/* leave interrupt */
  rt_interrupt_leave();    //���ж���һ��Ҫ������Ժ������뿪�ж�
}
#endif

#include <stdlib.h>
int return_int(char argv[])
{
	return atoi(argv);
}
float return_float(char argv[])
{
  return atof(argv);
}
