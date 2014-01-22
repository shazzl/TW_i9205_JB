/*
		G2TOUCH.INC
*/
#include <linux/workqueue.h>
#include <linux/firmware.h>
#include <linux/g2tsp_platform.h>
#include <linux/earlysuspend.h>
#include <linux/delay.h>
#include <linux/gpio.h>


#include "g2tsp.h"
#include "g2tsp_misc.h"


//#define G2TSP_USE_OLD_CHIP	0

void g2tsp_flash_eraseall(struct g2tsp *ts)
{
    int i;
	int err;

	g2debug1("%s %d\n", __func__, __LINE__);
	printk("G2TSP : FLASH ERASE ALL\n");

	ts->platform_data->reset();
    ts->bus_ops->write(ts->bus_ops, 0x1, 0x01);
    ts->bus_ops->write(ts->bus_ops, 0x0, 0x10); 	// soft reset
    ts->bus_ops->write(ts->bus_ops, 0x0, 0x00); 

    for(i=0;i<10;i++)
        udelay(1000);

    ts->bus_ops->write(ts->bus_ops, 0xcc, 0x80);   //partial Erase
    ts->bus_ops->write(ts->bus_ops, 0xc7, 0x98);

    udelay(1);

    for(i = 0; i < 10; i++)
    {
        ts->bus_ops->write(ts->bus_ops, 0xc7, 0xc8);
        ts->bus_ops->write(ts->bus_ops, 0xc7, 0xd8);
    }

    for(i = 0; i < 8; i++)
    {
        ts->bus_ops->write(ts->bus_ops, 0xc7, 0xcc);
        ts->bus_ops->write(ts->bus_ops, 0xc7, 0xdc);
    }

    // BIST Command Start
    for(i = 0; i < 7; i++)
    {
        ts->bus_ops->write(ts->bus_ops, 0xc7, 0xc5);
        ts->bus_ops->write(ts->bus_ops, 0xc7, 0xd5);
    }

    for(i = 0; i < 40; i++)
    {
        ts->bus_ops->write(ts->bus_ops, 0xc7, 0xc4);
        ts->bus_ops->write(ts->bus_ops, 0xc7, 0xd4);
    }

    for(i = 0; i < 4; i++)
    {
        ts->bus_ops->write(ts->bus_ops, 0xc7, 0xc5);
        ts->bus_ops->write(ts->bus_ops, 0xc7, 0xd5);
    }

    ts->bus_ops->write(ts->bus_ops, 0xc7, 0xc4);
    ts->bus_ops->write(ts->bus_ops, 0xc7, 0xd4);
    ts->bus_ops->write(ts->bus_ops, 0xc7, 0xc5);
    ts->bus_ops->write(ts->bus_ops, 0xc7, 0xd5);

    for(i = 0; i < 6; i++)
    {
        ts->bus_ops->write(ts->bus_ops, 0xc7, 0xc4);
        ts->bus_ops->write(ts->bus_ops, 0xc7, 0xd4);
    }

    // BIST Command End

    // Internal TCK set to SCL
    ts->bus_ops->write(ts->bus_ops, 0xc7, 0xcc);
    ts->bus_ops->write(ts->bus_ops, 0xc7, 0xdc);
    ts->bus_ops->write(ts->bus_ops, 0xc7, 0xfc);

		
	{
#if (G2TSP_NEW_IC == 1)		
		ts->platform_data->i2c_to_gpio(1);							// switch from I2C to GPIO
		
		err = gpio_request(ts->platform_data->gpio_scl, "SCL_");
		if (err)
			printk(KERN_ERR "#### failed to request SCL_ ####\n");
		
		err = gpio_request(ts->platform_data->gpio_sda, "SDA_");
		if (err)
			printk(KERN_ERR "#### failed to request SDA_ ####\n");
#endif

		gpio_direction_output(ts->platform_data->gpio_scl, 0);
		gpio_direction_output(ts->platform_data->gpio_sda, 0);
	
		udelay(1);
		gpio_set_value(ts->platform_data->gpio_sda, 0);
	    udelay(1);
		gpio_set_value(ts->platform_data->gpio_scl, 0);
	    udelay(1);
		gpio_set_value(ts->platform_data->gpio_scl, 1);
		udelay(1);
	}

    // Internal Flash Erase Operation Start
    
    for(i = 0; i < 1620000; i++)
    {
		gpio_set_value(ts->platform_data->gpio_scl, 0);
		gpio_set_value(ts->platform_data->gpio_scl, 1);			
    }

    // Internal Flash Erase Operation End
#if (G2TSP_NEW_IC == 0)
	{
		gpio_set_value(ts->platform_data->gpio_scl, 1);	
	}
#else
	{
		gpio_set_value(ts->platform_data->gpio_scl, 1);
		gpio_free(ts->platform_data->gpio_scl);
		gpio_free(ts->platform_data->gpio_sda);
   		 ts->platform_data->i2c_to_gpio(0);							// switch from GPIO to I2C
	}
#endif
    // SCL is returned to I2C, Mode Desable.
    ts->bus_ops->write(ts->bus_ops, 0xc7, 0x88);
    ts->bus_ops->write(ts->bus_ops, 0xc7, 0x00);



	// remark 2012.11.05
    ts->bus_ops->write(ts->bus_ops, 0xcc, 0x00);

		
    udelay(10);

	//current_ts->platform_data->reset();
    //current_ts->bus_ops->write(current_ts->bus_ops, 0x1, 0x1);
    //current_ts->bus_ops->write(current_ts->bus_ops, 0x0, 0x0);

    for(i=0;i<10;i++)
        udelay(1000);

	printk("G2TSP : FLASH ERASE Exit\n");
}

				   
void g2tsp_firmware_load(struct g2tsp 	*ts, const unsigned char *data, size_t size)
{
	int	i;
	int latest_version[0];
	
	latest_version[0] = ts->platform_data->fw_version[0];
	latest_version[1] = ts->platform_data->fw_version[1];

	printk("G2TOUCH: Curent device version = 0x%06x.%06x, latest version = 0x%06x.%06x \r\n",
					ts->current_firmware_version[0],ts->current_firmware_version[1], latest_version[0],latest_version[1]);

	if((ts->current_firmware_version[0] != latest_version[0])||(ts->current_firmware_version[1] != latest_version[1]))
	{
		printk("G2TOUCH: Firmware Update Start !!\r\n");

		// erase flash
		g2tsp_flash_eraseall(ts);

		
	 	// write binary
		ts->bus_ops->write(ts->bus_ops, 0xa1, 0x00);
		ts->bus_ops->write(ts->bus_ops, 0xa2, 0x00);
		ts->bus_ops->write(ts->bus_ops, 0xa3, 0x00);
		ts->bus_ops->write(ts->bus_ops, 0xa0, 0x01);

		for(i=0;i<size;i++)	{
			ts->bus_ops->write(ts->bus_ops, 0xa4, (data[i] & 0xff));
			ts->bus_ops->write(ts->bus_ops, 0xa5, 0x01);
			ts->bus_ops->write(ts->bus_ops, 0xa5, 0x00);
		}

		ts->bus_ops->write(ts->bus_ops, 0xa4, 0x00);
		ts->bus_ops->write(ts->bus_ops, 0xa5, 0x01);
		ts->bus_ops->write(ts->bus_ops, 0xa0, 0x00);
		ts->bus_ops->write(ts->bus_ops, 0xa5, 0x00);

		// remark 2012.11.05
		ts->bus_ops->write(ts->bus_ops, 0x01, 0x00);	//soft reset	
		ts->bus_ops->write(ts->bus_ops, 0x00, 0x00);	//soft reset	

		printk("\nG2TOUCH: written %d Bytes finished. \n", i);
		printk("G2TOUCH: Firmware Download Completed !! \n");
	}
	else
	{
		printk("G2TOUCH: current version is the latest version !! \r\n");
	}
	
	//ts->firmware_download = 0;
	return;
}


void firmware_request_handler(const struct firmware *fw, void *context)
{
	struct g2tsp 	*ts;
	ts = (struct g2tsp*) context;

	if(fw != NULL){
		g2tsp_firmware_load(ts, fw->data, fw->size);
		release_firmware(fw);
	} else {
        printk(KERN_ERR"failed to load G2TOuch firmware will not working\n");
    }
}







