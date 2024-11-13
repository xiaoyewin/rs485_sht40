
#include "flash.h"

/*******************************************************************************
* Function Name  : doseFlashHasPackedMessage
* Description    : Does flash has packed messages   
* Input          : None
* Output         : 
* Return         : ture/false
*******************************************************************************/
bool doseFlashHasPackedMessage(void)
{
    uint16_t length;
    uint16_t getHead;    

    /*Is head matched*/ 
    getHead = (uint16_t)(*(uint16_t*)(STM32F0xx_FLASH_PAGE15_STARTADDR ));      
    if( EEPPROM_PACKAGEHEAD != getHead )
    {
        return false;
    }
    
    /*Is length zero*/
    length = (*(uint16_t*)(STM32F0xx_FLASH_PAGE15_STARTADDR+2));
    if( 0 == length)
    {
        return false;
    }
    
    return true;
}
/*******************************************************************************
* Function Name  : getValuablePackedMessageLengthofFlash
* Description    : Get valuable packed message length of flash 
* Input          : None
* Output         : 
* Return         : valuable length
*******************************************************************************/
uint16_t getValuablePackedMessageLengthofFlash( void )
{
    uint16_t length;
         
    /*Is head matched*/       
    if( EEPPROM_PACKAGEHEAD != (*(uint16_t*)(STM32F0xx_FLASH_PAGE15_STARTADDR )) )
    {
        return 0;
    }
    
    /*Get length*/
    length = (uint16_t)(*(uint16_t*)(STM32F0xx_FLASH_PAGE15_STARTADDR+2));   
    
    return length;
}
/*******************************************************************************
* Function Name  : readPackedMessageFromFlash
* Description    : Read packed message form flash
* Input          : buff:point to first location of received buffer.length:Maxmum length of reception
* Output         : 
* Return         : reception length
*******************************************************************************/
uint16_t readPackedMessageFromFlash( uint8_t *buff , uint16_t length)
{
    int i;
    uint16_t getLength;
    
    if( !doseFlashHasPackedMessage() )
        return 0;
    
    /*Get valuable length*/
    getLength = getValuablePackedMessageLengthofFlash();
    
    /*Read out message*/
    for(i=0;i<MIN(getLength,length);i++)
    {
        buff[i]= *(uint8_t*)(STM32F0xx_FLASH_PAGE15_STARTADDR+4+i);
    }     
    
    return MIN(getLength,length);
}
/*******************************************************************************
* Function Name  : isItOddNumber
* Description    : is input data an odd number?
* Input          : number:input data
* Output         : 
* Return         : true/false
*******************************************************************************/
bool isItOddNumber(uint16_t number)
{
    if(0 != number%2)
    {
        return true;
    }
    return false;
}
/*******************************************************************************
* Function Name  : Flash_eeprom_WriteWithPacked
* Description    : Write a group of datas to flash.
* Input          : buff:pointer of first data, length: write length
* Output         : 
* Return         : true/false
*******************************************************************************/
bool writeMessageToFlash( uint8_t *buff , uint16_t length)
{
    uint16_t temp;
    int i;
    
    /*Protection*/
    if((length+4) > STM32F0xx_PAGE_SIZE )
    {
        return false;
    }
    
    FLASH_Unlock(  );    

    /*Clear all flags*/
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR );
    
    /*Erase first . Do not rember.*/
    if(FLASH_COMPLETE != FLASH_ErasePage(STM32F0xx_FLASH_PAGE15_STARTADDR))//¡ã¨¹o?¨¢?¦Ì¨¨¡äybusy
    {
        return false;
    }
    
    /*Write head*/
    FLASH_ProgramHalfWord( STM32F0xx_FLASH_PAGE15_STARTADDR, EEPPROM_PACKAGEHEAD );
    /*Write length*/
    FLASH_ProgramHalfWord( STM32F0xx_FLASH_PAGE15_STARTADDR+2 , length );
    
    
    /*Write datas*/
    for(i=0 ;i<length/2 ;i++)
    {
        temp = buff[2*i]|(uint16_t)buff[2*i+1]<<8;
        FLASH_ProgramHalfWord( STM32F0xx_FLASH_PAGE15_STARTADDR+4+2*i , temp);
    }  
    if( isItOddNumber(length) )//Write one more if length is odd number.
    {        
        temp = (uint16_t)buff[length-1];
        FLASH_ProgramHalfWord( STM32F0xx_FLASH_PAGE15_STARTADDR+4+(length-1) , temp);
    }

    
    /*Read out and check*/
    for(i=0 ;i<length ;i++)
    {
        if( *(uint8_t*)(STM32F0xx_FLASH_PAGE15_STARTADDR+4+i) != buff[i] )
        {
            FLASH_Lock();
            return false;
        }
    }    
    
    FLASH_Lock();
    return true;    
}


/*******************************************************************************
* Function Name  : flashReadWriteTest
* Description    : Flash read write test.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void flashReadWriteTest( void ) 
{
    #define testReadWriteNumber  200
    uint8_t buff_write[testReadWriteNumber]={0};
    uint8_t buff_read[testReadWriteNumber]={0};
    uint16_t length;
    int i;
    
    for( i=0;i<testReadWriteNumber;i++)
    {
        buff_write[i]=i;
    }
    
    writeMessageToFlash( buff_write , testReadWriteNumber);
    length = readPackedMessageFromFlash( buff_read , testReadWriteNumber);   

}