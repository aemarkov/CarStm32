#include "uart.h"

//////////////////// МИНИ-РЕАЛИЗАЦИЯ КОЛЬЦЕВОЙ ОЧЕРЕДИ /////////////////////////

typedef struct
{
    uint8_t Buffer[BLUETOOTH_USART_SEND_BUFFER_SIZE];
    uint8_t Head;
    uint8_t Tail;
} Queue;

Queue _sendQueue;

// Инициализация
void Queue_Init(Queue* queue)
{
   queue->Head = 0;
   queue->Tail = 0;
}

// Добавление в очередь
void Queue_Push(Queue* queue, uint8_t byte)
{
    queue->Buffer[queue->Head] = byte;
    queue->Head++;
    if(queue->Head>BLUETOOTH_USART_SEND_BUFFER_SIZE)
        queue->Head = 0;
}

// Извлечение из очереди
uint8_t Queue_Pull(Queue* queue)
{
    uint8_t c = queue->Buffer[queue->Tail];
    queue->Tail++;
    if(queue->Tail > BLUETOOTH_USART_SEND_BUFFER_SIZE)
        queue->Tail = 0;
    
    return c;
}

// Пуста ли очередь
bool Queue_IsEmpty(Queue* queue)
{
    return queue->Head == queue->Tail;
}

///////////////////////////////////////////////////////////////////////////////


// Для парсинга
typedef enum
{
    L_START,
    L_VALUE,
    L_END,
    R_START,
    R_VALUE,
    R_END,
    PARSE_ERROR
} ParseState; 

#define BUFFER_SIZE 10

ParseState _parseState;
char _buffer[BUFFER_SIZE];
int _bufferIndex;
int16_t _leftValue, _rightValue;

// Обработчик функции
CommandHandler _uartCommandHandler;


///////////////////////////////////////////////////////////////////////////////


void UartInit(int baud, CommandHandler handler)
{
    RCC_AHBPeriphClockCmd(
        RCC_AHBPeriph_GPIOB |
        RCC_AHBPeriph_GPIOE,
		ENABLE);
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
    
    
    //Настройка пинов USART
	GpioInitAF(BLUETOOTH_USART_TX_GPIO, BLUETOOTH_USART_TX_PIN, GPIO_OType_PP, GPIO_PuPd_NOPULL, GPIO_Speed_Level_1);
	GpioInitAF(BLUETOOTH_USART_RX_GPIO, BLUETOOTH_USART_RX_PIN, GPIO_OType_PP, GPIO_PuPd_NOPULL, GPIO_Speed_Level_1);

    GPIO_PinAFConfig(BLUETOOTH_USART_TX_GPIO, GPIO_PinSource10, GPIO_AF_7);
    GPIO_PinAFConfig(BLUETOOTH_USART_RX_GPIO, GPIO_PinSource15, GPIO_AF_7);
    
    _parseState = L_START;
    _leftValue = 0;
    _rightValue = 0;
    _bufferIndex = 0;
    _uartCommandHandler = handler;
    
	//Настройка USART
	USART_InitTypeDef usartInit;
	usartInit.USART_BaudRate = baud;
	usartInit.USART_WordLength = USART_WordLength_8b;
	usartInit.USART_StopBits = USART_StopBits_1;
	usartInit.USART_Parity = USART_Parity_No;
	usartInit.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	usartInit.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	
	USART_DeInit(BLUETOOTH_USART);
	USART_Init(BLUETOOTH_USART, &usartInit);
	USART_Cmd(BLUETOOTH_USART, ENABLE);
	

    Queue_Init(&_sendQueue);

	//Настройка прерываний USART	
	NVIC_InitTypeDef init;
	init.NVIC_IRQChannel = BLUETOOTH_USART_IRQn;
	init.NVIC_IRQChannelPreemptionPriority  = 11;
	init.NVIC_IRQChannelSubPriority = 0;
	init.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&init);
	
	USART_ITConfig(BLUETOOTH_USART, USART_IT_RXNE, ENABLE);
}


void UartSend(uint8_t* data, uint8_t length)
{    
    Queue_Push(&_sendQueue, 76);
    uint8_t parity = 0;
    
    for(uint8_t i = 0; i<length; i++)
    {
        Queue_Push(&_sendQueue, data[i]);
        parity += data[i];
    }
    
    Queue_Push(&_sendQueue, parity);
    
    USART_ITConfig(BLUETOOTH_USART, USART_IT_TXE, ENABLE);
}


// Обработчик прерывания
void BLUETOOTH_USART_IRQHandler()
{   
    if(USART_GetITStatus(BLUETOOTH_USART, USART_IT_TXE)!=RESET)
	{
        // Передача
        
        USART_ClearITPendingBit(BLUETOOTH_USART, USART_IT_TXE);
        
        if(Queue_IsEmpty(&_sendQueue))
        {
            // Очередь заполнена
            USART_ITConfig(BLUETOOTH_USART, USART_IT_TXE, DISABLE);  
            return;
        }
        
        uint8_t c = Queue_Pull(&_sendQueue);
        USART_SendData(BLUETOOTH_USART, c);
        
        
    }
    else if(USART_GetITStatus(BLUETOOTH_USART, USART_IT_RXNE)!=RESET)
	{
        //Прием
        
        USART_ClearITPendingBit(BLUETOOTH_USART, USART_IT_RXNE);
        char c = (char)USART_ReceiveData(BLUETOOTH_USART);
        
        /* Формат:
            L<value>\n\rR<value>\n\r
        */
        
        // Парсим команды сраным автоматом, потому что кто-то любит команды
        // в виде текста
        if(c=='L')
        {
            _parseState = L_VALUE;
            _bufferIndex = 0;
            return;
        }
        if(c=='R')
        {
            _parseState = R_VALUE;
            _bufferIndex = 0;
            return;
        }
        
        if(_parseState == L_VALUE)
        {
            // значение левого двигателя 
            if(c!='\r')
            {
                _buffer[_bufferIndex] = c;
                _bufferIndex++;
                
                if(_bufferIndex >= BUFFER_SIZE)
                {
                    _parseState = PARSE_ERROR;
                }
            }
            else
            {
                 //Парсинг левого закончен
                _buffer[_bufferIndex] = '\0';
                _leftValue = strtol(_buffer, NULL, 10);
                _parseState = L_END;
            }            
        }
        else if(_parseState == R_VALUE)
        {
            // значение правого двигателя 
            if(c!='\r')
            {
                _buffer[_bufferIndex] = c;
                _bufferIndex++;
                
                if(_bufferIndex >= BUFFER_SIZE)
                {
                    _parseState = PARSE_ERROR;
                }
            }
            else
            {
                //Парсинг правого закончен
                _buffer[_bufferIndex] = '\0';
                _rightValue = strtol(_buffer, NULL, 10);
                _parseState = R_END;
                
                _uartCommandHandler(_leftValue, _rightValue);
            }
        }
    }
}
