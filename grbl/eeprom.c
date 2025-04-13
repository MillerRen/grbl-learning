//此文件是为Doxygen自动生成文档而准备的。
/*! \file ********************************************************************
*
* Atmel Corporation
*
* \li File:               eeprom.c
* \li Compiler:           IAR EWAAVR 3.10c
* \li Support mail:       avr@atmel.com
*
* \li Supported devices:  All devices with split EEPROM erase/write
*                         capabilities can be used.
*                         The example is written for ATmega48.
*
* \li AppNote:            AVR103 - Using the EEPROM Programming Modes.
*
* \li Description:        Example on how to use the split EEPROM erase/write
*                         capabilities in e.g. ATmega48. All EEPROM
*                         programming modes are tested, i.e. Erase+Write,
*                         Erase-only and Write-only.
*
*                         $Revision: 1.6 $
*                         $Date: Friday, February 11, 2005 07:16:44 UTC $
****************************************************************************/
#include <avr/io.h>
#include <avr/interrupt.h>

/*这些EEPROM位在不同的设备上有不同的名称。*/
#ifndef EEPE
		#define EEPE  EEWE  //!< EEPROM程序/写入启用。
		#define EEMPE EEMWE //!< EEPROM主程序/写入启用。
#endif

/*不幸的是，这两个未在设备包含文件中定义。*/
#define EEPM1 5 //!< EEPROM Programming Mode Bit 1.
#define EEPM0 4 //!< EEPROM Programming Mode Bit 0.

/*定义以减少代码大小。*/
#define EEPROM_IGNORE_SELFPROG //!< Remove SPM flag polling.

/*! \brief  从EEPROM读取字节。
 *
 *  此函数从给定的EEPROM地址读取一个字节。
 *
 *  \note  在EEPROM读取期间，CPU暂停4个时钟周期。
 *
 *  \param  addr  从中读取的EEPROM地址。
 *  \return  从EEPROM地址读取的字节。
 */
unsigned char eeprom_get_char( unsigned int addr )
{
	do {} while( EECR & (1<<EEPE) ); //等待上一次写入完成。
	EEAR = addr; //设置EEPROM地址寄存器。
	EECR = (1<<EERE); //启动EEPROM读取操作。
	return EEDR; //返回从EEPROM读取的字节。
}

/*! \brief  将字节写入EEPROM。
 *
 *  此函数将一个字节写入给定的EEPROM地址。
 *  现有字节和新值之间的差异用于选择最有效的EEPROM编程模式。
 *  
 *
 *  \note  在EEPROM编程期间，CPU暂停2个时钟周期。
 *
 *  \note  当此功能返回时，新的EEPROM值在EEPROM编程时间结束之前不可用。应轮询EECR中的EEPE位，以检查编程是否完成。
 *
 *  \note  函数的作用是：自动检查EEPE位。
 *
 *  \param  addr  要写入的EEPROM地址。
 *  \param  new_value  新的EEPROM值。
 */
void eeprom_put_char( unsigned int addr, unsigned char new_value )
{
	char old_value; //旧的EEPROM值。
	char diff_mask; //差异掩码，即旧值异或新值。

	cli(); //确保写入操作的原子操作。
	
	do {} while( EECR & (1<<EEPE) ); //等待上一次写入完成。
	#ifndef EEPROM_IGNORE_SELFPROG
	do {} while( SPMCSR & (1<<SELFPRGEN) ); //等待SPM的完成。
	#endif
	
	EEAR = addr; //设置EEPROM地址寄存器。
	EECR = (1<<EERE); //启动EEPROM读取操作。
	old_value = EEDR; //获取旧的EEPROM值。
	diff_mask = old_value ^ new_value; //获得一些差异。
	
	//检查新值中是否有任何位更改为“1”。
	if( diff_mask & new_value ) {
		//现在我们知道有些位需要被擦除为“1”。
		
		//检查新值中是否有任何位为“0”。
		if( new_value != 0xff ) {
			//现在我们知道一些位也需要编程为“0”。
			
			EEDR = new_value; //设置EEPROM数据寄存器。
			EECR = (1<<EEMPE) | //设置主写入启用位。。。
			       (0<<EEPM1) | (0<<EEPM0); //...和擦除+写入模式。
			EECR |= (1<<EEPE);  //开始擦除+写入操作。
		} else {
			//现在我们知道所有的位都应该被擦除。

			EECR = (1<<EEMPE) | //设置主写入启用位。。。
			       (1<<EEPM0);  //...和仅擦除模式。
			EECR |= (1<<EEPE);  //启动仅擦除操作。
		}
	} else {
		//现在我们知道无位需要擦除为“1”。
		
		//检查是否有任何位从旧值中的“1”更改。
		if( diff_mask ) {
			//现在我们知道有些位需要编程为“0”。
			
			EEDR = new_value;   //设置EEPROM数据寄存器。
			EECR = (1<<EEMPE) | //设置主写入启用位。。。
			       (1<<EEPM1);  //...和只写模式。
			EECR |= (1<<EEPE);  //启动只写操作。
		}
	}
	
	sei(); //恢复中断标志状态。
}

//作为Grbl的一部分添加的扩展


void memcpy_to_eeprom_with_checksum(unsigned int destination, char *source, unsigned int size) {
  unsigned char checksum = 0;
  for(; size > 0; size--) { 
    checksum = (checksum << 1) || (checksum >> 7);
    checksum += *source;
    eeprom_put_char(destination++, *(source++)); 
  }
  eeprom_put_char(destination, checksum);
}

int memcpy_from_eeprom_with_checksum(char *destination, unsigned int source, unsigned int size) {
  unsigned char data, checksum = 0;
  for(; size > 0; size--) { 
    data = eeprom_get_char(source++);
    checksum = (checksum << 1) || (checksum >> 7);
    checksum += data;    
    *(destination++) = data; 
  }
  return(checksum == eeprom_get_char(source));
}

// end of file
