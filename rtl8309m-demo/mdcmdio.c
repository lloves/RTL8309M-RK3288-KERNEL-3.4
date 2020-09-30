/*
* Copyright (C) 2012 Realtek Semiconductor Corp.
* All Rights Reserved.
*
* This program is the proprietary software of Realtek Semiconductor
* Corporation and/or its licensors, and only be used, duplicated,
* modified or distributed under the authorized license from Realtek.
*
* ANY USE OF THE SOFTWARE OTEHR THAN AS AUTHORIZED UNDER 
* THIS LICENSE OR COPYRIGHT LAW IS PROHIBITED.
* 
* $Revision: v1.0.1 $
* $Date: 2012-10-23 11:18:41 +0800 $
*
* Purpose : MDC/MDIO example code
*
* Feature :  This file consists of following modules:
*                
*
*/
#include <stdio.h>  
#include <fcntl.h>  
#include <stdlib.h>  
#include <pthread.h> 
#include <sys/ioctl.h>

//#include "rtl_types.h"
#include "mdcmdio.h"      /*RTL8651B file*/
//#include "asicRegs.h"
#include "rtl8309n_types.h"

#define RTL8309_MAGIC 'R' 
#define RTL8309M_GET_REG_DATA   _IO(RTL8309_MAGIC, 0x20)
#define RTL8309M_SET_REG_DATA   _IO(RTL8309_MAGIC, 0x21)
#define RTL8309M_READ_REG       _IO(RTL8309_MAGIC, 0x22)
#define RTL8309M_WRITE_REG      _IO(RTL8309_MAGIC, 0x23)

struct rtl8309m_register_data {
	unsigned int phyad;
	unsigned int regad;
};

int32 fd;
pthread_mutex_t mutex;


/* Function Name:
 *      smiRead
 * Description:
 *      Read data from phy register
 * Input:
 *      phyad   - PHY address (0~31)
 *      regad   -  Register address (0 ~31) 
 * Output:
 *      data    -  Register value 
 * Return:
 *      SUCCESS         -  Success
 *      FAILED            -  Failure
 * Note:
 *     This function could read register through MDC/MDIO serial 
 *     interface, and it is platform  related. It use two GPIO pins 
 *     to simulate MDC/MDIO timing. MDC is sourced by the Station Management 
 *     entity to the PHY as the timing reference for transfer of information
 *     on the MDIO signal. MDC is an aperiodic signal that has no maximum high 
 *     or low times. The minimum high and low times for MDC shall be 160 ns each, 
 *     and the minimum period for MDC shall be 400 ns. Obeying frame format defined
 *     by IEEE802.3 standard, you could access Phy registers. If you want to 
 *     port it to other CPU, please modify static functions which are called 
 *      by this function.
 */
int32 smiRead(uint32 phyad, uint32 regad, uint32 * data) 
{
	struct rtl8309m_register_data reg_data;
	int32 ret;
	
    if ((phyad > 8) || (regad > 31) || (data == NULL))  
        return FAILED;

    /*it lock the resource to ensure that SMI opertion is atomic, 
     *the API is based on RTL865X, it is used to disable CPU interrupt,
     *if porting to other platform, please rewrite it to realize the same function
     */

    pthread_mutex_lock(&mutex);   

    reg_data.phyad = phyad;
    reg_data.regad = regad;
	
    ret = ioctl(fd, RTL8309M_SET_REG_DATA, &reg_data);
    if (ret == -1){
    	perror("Realtek:ioctl set reg data fail.");
    	return FAILED;
    }
    
    ret = ioctl(fd, RTL8309M_READ_REG, data);
	if (ret == -1){
    	perror("Realtek:ioctl read reg fail.");
    	return FAILED;
    }

    /*unlock the source, enable interrupt*/    
    pthread_mutex_unlock(&mutex);
    
    return  SUCCESS;
}

/* Function Name:
 *      smiWrite
 * Description:
 *      Write data to Phy register
 * Input:
 *      phyad   - PHY address (0~31)
 *      regad   -  Register address (0 ~31)
 *      data    -  Data to be written into Phy register
 * Output:
 *      none
 * Return:
 *      SUCCESS         -  Success
 *      FAILED            -  Failure
 * Note:
 *     This function could read register through MDC/MDIO serial 
 *     interface, and it is platform  related. It use two GPIO pins 
 *     to simulate MDC/MDIO timing. MDC is sourced by the Station Management 
 *     entity to the PHY as the timing reference for transfer of information
 *     on the MDIO signal. MDC is an aperiodic signal that has no maximum high 
 *     or low times. The minimum high and low times for MDC shall be 160 ns each, 
 *     and the minimum period for MDC shall be 400 ns. Obeying frame format defined
 *     by IEEE802.3 standard, you could access Phy registers. If you want to 
 *     port it to other CPU, please modify static functions which are called 
*      by this function.
 */

int32 smiWrite(uint32 phyad, uint32 regad, uint32 data)
{
    struct rtl8309m_register_data reg_data;
	int32 ret;
	uint32 value;

    if ((phyad > 31) || (regad > 31) || (data > 0xFFFF))  
        return FAILED;

    /*it lock the resource to ensure that SMI opertion is atomic, 
      *the API is based on RTL865X, it is used to disable CPU interrupt,
      *if porting to other platform, please rewrite it to realize the same function
      */
    pthread_mutex_lock(&mutex);   

    reg_data.phyad = phyad;
    reg_data.regad = regad;
	value = data;
	
    ret = ioctl(fd, RTL8309M_SET_REG_DATA, &reg_data);
    if (ret == -1){
    	perror("Realtek:ioctl set reg data fail.");
    	return FAILED;
    }
    
    ret = ioctl(fd, RTL8309M_WRITE_REG, &value);
	if (ret == -1){
    	perror("Realtek:ioctl write reg fail.");
    	return FAILED;
    }

    /*unlock the source, enable interrupt*/        
    pthread_mutex_unlock(&mutex);
            
    return SUCCESS; 
}


/* Function Name:
 *      smiInit
 * Description:
 *      Init Rtl8651B smi interface
 * Input:
 *      void
 * Output:
 *      none
 * Return:
 *      SUCCESS         -  Success
 *      FAILED            -  Failure
 */
int32 smiInit(void)
{	
	fd = open("/dev/rtl8309m", O_RDWR);
    if (fd == -1){
    	perror("Realtek: device open fail.");
		return FAILED;
    }

	pthread_mutex_init(&mutex,NULL);

	return SUCCESS;
}

/* Function Name:
 *      smiRelease
 * Description:
 *      Release Rtl8651B smi interface
 * Input:
 *      void
 * Output:
 *      none
 * Return:
 *      SUCCESS         -  Success
 *      FAILED            -  Failure
 * Note:
 *      none
 */
void smiRelease(void)
{
	close(fd);
	pthread_mutex_destroy(&mutex);
}

/* Function Name:
 *      smiReadBit
 * Description:
 *      Read one bit of PHY register
 * Input:
 *      phyad   - PHY address (0~31)
 *      regad   -  Register address (0 ~31) 
 *      bit       -  Register bit (0~15)   
 * Output:
 *      pdata    - the pointer of  Register bit value 
 * Return:
 *      SUCCESS         -  Success
 *      FAILED            -  Failure
 * Note:
 */

int32 smiReadBit(uint32 phyad, uint32 regad, uint32 bit, uint32 * pdata) 
{
    uint32 regData;

    if ((phyad > 31) || (regad > 31) || (bit > 15) || (pdata == NULL) ) 
        return  FAILED;
    
    if(bit>=16)
        * pdata = 0;
    else 
    {
        smiRead(phyad, regad, &regData);
        if(regData & (1<<bit)) 
            * pdata = 1;
        else
            * pdata = 0;
    }
    return SUCCESS;
}

/* Function Name:
 *      smiWriteBit
 * Description:
 *      Write one bit of PHY register
 * Input:
 *      phyad   - PHY address (0~31)
 *      regad   -  Register address (0 ~31) 
 *      bit       -  Register bit (0~15)   
 *      data     -  Bit value to be written
 * Output:
 *      none
 * Return:
 *      SUCCESS         -  Success
 *      FAILED            -  Failure
 * Note:
 */

int32 smiWriteBit(uint32 phyad, uint32 regad, uint32 bit, uint32 data) 
{
    uint32 regData;
    
    if ((phyad > 31) || (regad > 31) || (bit > 15) || (data > 1) ) 
        return  FAILED;
    smiRead(phyad, regad, &regData);
    if(data) 
        regData = regData | (1<<bit);
    else
        regData = regData & ~(1<<bit);
    smiWrite(phyad, regad, regData);
    return SUCCESS;
}




















