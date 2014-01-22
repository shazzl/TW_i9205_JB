
/* linux/driver/input/touchscreen/g2tsp.c
 *
 * Copyright (c) 2009-2012 G2Touch Co., Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 * 
 *	# history # 
 *	- 2012.11.09, by jhkim
        : add Auto firmware download.
*/

#include <linux/delay.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/byteorder/generic.h>
#include <linux/bitops.h>
#include <linux/earlysuspend.h>
#include <linux/g2tsp_platform.h>
#include <linux/miscdevice.h>
#include <linux/delay.h>
#include <linux/firmware.h>

#include <asm/system.h>
#include <asm/uaccess.h>

#include "g2tsp.h"

#include "g2tsp_misc.h"

#if (G2TSP_NEW_IC == 0)
#include "g2touch_i2c.h"
#endif

#ifdef CONFIG_LEDS_CLASS
#include <linux/regulator/consumer.h>

#include <mach/vreg.h>
#include <mach/pmic.h>
#endif

#define DEF_TRACKING_ID	10	

#define G2_REG_TOUCH_BASE   0x60
#define G2TSP_FW_NAME		"g2tsp_fw.bin"

static struct delayed_work 	fw_work;
#define FW_RECOVER_DELAY	msecs_to_jiffies(500)

struct g2tsp_trk {
	unsigned touch_id_num:4;
	unsigned sfk_id_num:4;
	
	unsigned fr_num:4;
	unsigned af1:1;
	unsigned rnd:1;
	unsigned sfkrnd:1;
	unsigned padding:1;

	unsigned y_h:3;
	unsigned x_h:3;
	unsigned area_h:2;

	u8 x_l;
	u8 y_l;
	u8 area_l;
}  __attribute((packed)); 

struct g2tsp_reg_data {
	unsigned tfnum:4;	
	unsigned sfknum:4; 		//special function key
	struct g2tsp_trk trk[10];	
} __attribute((packed));

struct g2tsp *current_ts;
const struct firmware *fw_info;
static 	unsigned char g2tsp_init_reg[4][2] = {
// {0x00, 0x01},
 {0x00, 0x10},
 {0x02, 0x21}, 
 {0x30, 0x18},
// {0xC6, 0x28},
 {0x00, 0x00},
};

static unsigned char g2tsp_suspend_reg[1][2] = {
 {0x00, 0x01}
};

static unsigned char g2tsp_resume_reg[1][2] = {
	 {0x00, 0x00}
};
#define G2TSP_INIT_REGS (sizeof(g2tsp_init_reg) / sizeof(g2tsp_init_reg[0]))
#define G2TSP_SUSPEND_REGS (sizeof(g2tsp_suspend_reg) / sizeof(g2tsp_suspend_reg[0]))
#define G2TSP_RESUME_REGS (sizeof(g2tsp_resume_reg) / sizeof(g2tsp_resume_reg[0]))

#ifdef CONFIG_LEDS_CLASS
static void msm_tkey_led_vdd_on(bool onoff)
{
	int ret;
	static struct regulator *reg_l36;

	if (!reg_l36) {
		reg_l36 = regulator_get(NULL, "8917_l36");
		if (IS_ERR(reg_l36)) {
			pr_err("could not get 8917_l36, rc = %ld=n",
				PTR_ERR(reg_l36));
			return;
		}
		ret = regulator_set_voltage(reg_l36, 3300000, 3300000);
		if (ret) {
			pr_err("%s: unable to set ldo36 voltage to 3.3V\n",
				__func__);
			return;
		}
	}

	if (onoff) {
		if (!regulator_is_enabled(reg_l36)) {
			ret = regulator_enable(reg_l36);
			if (ret) {
				pr_err("enable l36 failed, rc=%d\n", ret);
				return;
			}
			pr_info("keyled 3.3V on is finished.\n");
		} else
			pr_info("keyled 3.3V is already on.\n");
	} else {
		if (regulator_is_enabled(reg_l36)) {
			ret = regulator_disable(reg_l36);
		if (ret) {
			pr_err("disable l36 failed, rc=%d\n", ret);
			return;
		}
		pr_info("keyled 3.3V off is finished.\n");
		} else
			pr_info("keyled 3.3V is already off.\n");
	}
}

static void msm_tkey_led_set(struct led_classdev *led_cdev,
	enum led_brightness value)
{
	bool tkey_led_on;

	if (value)
		tkey_led_on = true;
	else
		tkey_led_on = false;

	msm_tkey_led_vdd_on(tkey_led_on);
}
#endif

static void check_firmware_version(struct g2tsp *ts, char *buf)
{
    printk("G2TSP :  Firmware Version !! \r\n");
    ts->current_firmware_version[0] = (buf[1]<<16) |(buf[2]<<8) |  buf[3];
    ts->current_firmware_version[1] = (buf[4]<<16) |(buf[5]<<8) |  buf[6];
    printk("G2TSP : fw_ver[0] = %06x, fw_ver[1] = %06x \r\n",  ts->current_firmware_version[0],ts->current_firmware_version[1]);

	if (ts->firmware_download)
	{

		
		request_firmware_nowait(THIS_MODULE,
	                      FW_ACTION_HOTPLUG,
                    	  G2TSP_FW_NAME,
                	      ts->pdev,
            	          GFP_KERNEL,
        	              ts,
    	                  firmware_request_handler);

		ts->firmware_download = 0;
	}

}


/* TSP Debug 데이터를 I2C로 받는 부분  */
// 총 60개 Byte중 49개 사용 가능.
static void MakeI2cDebugBuffer(u8 *dst, u8 *src, u8 fcnt)
{
	u16 i,pos=0;
    u8  *temp;
	
	temp = dst;
    pos = (fcnt * 49);

    for(i=0; i<10; i++)
    {
        if(i != 1) {
            temp[pos++] = src[(i*6)];
        }
        
        temp[pos++] = src[(i*6)+2];
        temp[pos++] = src[(i*6)+3];
        temp[pos++] = src[(i*6)+4];
        temp[pos++] = src[(i*6)+5];
    }
}


/* 한번에 모든 데이터를 받지 않고 60Byte  Packet 단위로 받아 들인다. 그 중 11Byte는 Read가 제대로 되지 않아 버리고 49개의 Byte만 실제로 사용 가능 하다. */
static void ReadTSPDbgData(struct g2tsp *ts, u8 type) 
{
	u8 buf[61];
	u8 maxFrameCnt, frameCnt = 0;
    
    static u8 	frameBuf[1024];	

	ts->bus_ops->read_blk(ts->bus_ops, G2_REG_TOUCH_BASE+1,60, buf); 
    
	maxFrameCnt = buf[1]& 0x1F; 		//frameCnt (0 ~ 12)
    frameCnt    = buf[7]& 0x1F;

	if(frameCnt > maxFrameCnt)  {
        ts->bus_ops->write(ts->bus_ops,0xC1, 0x01);

		return;
	}
  
    if(type == 0xF2)
    {
        MakeI2cDebugBuffer(frameBuf,buf,frameCnt);
        
        if(frameCnt == (maxFrameCnt-1)) 
        {
            u16 len, checkSum=0, i, bufCheckSum;
            len = (frameBuf[3]<<8) | (frameBuf[4]);

            for(i=0; i<(len+7); i++)  {
                checkSum += frameBuf[i];
            }
            bufCheckSum = (frameBuf[len+7] << 8) | (frameBuf[len+8]);
            
            if(bufCheckSum != checkSum)
            {
                printk("[G2TSP]Packet Err: len=%d, checkSum = 0x%04x, bufCheckSum = 0x%02x%02x \r\n",len+7, checkSum, frameBuf[len+7],frameBuf[len+8]);
            }
            else
            {
                //UartWrite(UART_1,I2cDbgBuf, len + 7);
                printk("[G2TSP] : Get Frame Data --> CMD: 0x%02x, length = %d \r\n", frameBuf[2], len+7);
            }
		}
    }
	
    ts->bus_ops->write(ts->bus_ops,0xC1, 0x01);

}



static void g2tsp_event_process(struct g2tsp *ts, u8 type)
{
    u8 buf[60];

    printk(" ### Event Type = %02x \r\n", type);
    
    switch(type)
    {
    case 0xF1:
        memset(buf,0,60);
    	if(ts->bus_ops->read_blk(ts->bus_ops, G2_REG_TOUCH_BASE,30, buf) >= 0) {      
	    	check_firmware_version(ts,buf);
        }        
        break;
        
    case 0xF2:
        ReadTSPDbgData(ts,type);
        break;
        
    default:
        
        break;
    }
    
}


static void g2tsp_input_worker(struct g2tsp *ts)
{
	struct g2tsp_reg_data rd;
	u8	touchNum,touchID, touchUpDown;
	u8 	sfkNum,sfkID,sfkUpDn;
    u16 x, y, area;
	int ret;
	int i, keycnt;

    memset((u8 *)&rd, 0, 61);

	ret = ts->bus_ops->read(ts->bus_ops,G2_REG_TOUCH_BASE,(u8 *)&rd);

    if(rd.sfknum == 0xf)
    {
        g2tsp_event_process(ts, *((u8*)&rd));
        return;
    }
   
    
	if((rd.tfnum != 0) || (rd.sfknum != 0))
	{

		keycnt = (rd.tfnum)? rd.tfnum : rd.sfknum ;

		if((rd.tfnum != 0) && (rd.sfknum != 0))
			goto error_worker;
		
		if(keycnt > 5)
			goto error_worker;
		
		if(keycnt)
		{
		    u8  *rdData;
		    u8 checkSum=0,tspCheckSum;
            
			ret = ts->bus_ops->read_blk(ts->bus_ops, G2_REG_TOUCH_BASE,(keycnt*6)+1, (u8 *)&rd); 

            if (ret < 0){
                //printk(KERN_ERR "%s : Cannot read i2c data\n", __FUNCTION__);
                goto error_worker;
            }


            rdData = (u8 *)&rd;
            
            for(i=0; i< ((keycnt*6) + 1); i++)
            {
                checkSum += *rdData++;
            }
            
            ts->bus_ops->read(ts->bus_ops,0xC2, &tspCheckSum);


            if(checkSum != tspCheckSum)
            {
                printk("checkSum Err!! deviceCheckSum = %02x, tspCheckSum = %02x \r\n", checkSum, tspCheckSum);
                goto error_worker;
            }
            // clear before checkSum
            ts->bus_ops->write(ts->bus_ops,0xC2, 0);
            
		}
	}
	else
	{
		return;
	}

	
	if (rd.tfnum)
	{
		//touchCnt;
		touchNum = rd.tfnum;

		for (i=0;i< touchNum;i++)
		{
			touchID = rd.trk[i].touch_id_num;
			x = (rd.trk[i].x_h << 8) | rd.trk[i].x_l;
			y = (rd.trk[i].y_h << 8) | rd.trk[i].y_l;
			area = (rd.trk[i].area_h << 8) | rd.trk[i].area_l;
			touchUpDown = rd.trk[i].rnd;
			//printk("%s %d ID:%d\t x:%d\t y:%d\t area:%d\t updn:%d\n", __func__, __LINE__, touchID, x, y, area, touchUpDown);
			if (ts->x_rev == 1)	
				x = ts->platform_data->res_x - x;

			if (ts->y_rev == 1)
				y = ts->platform_data->res_y - y;

			if(x > ts->platform_data->res_x)	
				return;
			if(y > ts->platform_data->res_y)
				return;
			g2debug2("%s %d ID:%d\t x:%d\t y:%d\t area:%d\t updn:%d\n", __func__, __LINE__, touchID, x, y, area, touchUpDown);
			input_mt_slot(ts->input, touchID + 1);		// event touch ID
			
			if (touchUpDown)
			{
				if(ts->prev_Area[touchID] > 0)
					area = (ts->prev_Area[touchID]*19+ area)/20;

				if (area < 1) area = 1;
				
				ts->prev_Area[touchID] = area;

				input_report_abs(ts->input, ABS_MT_TRACKING_ID, touchID + 1);
                input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, area);   // press       
                input_report_abs(ts->input, ABS_MT_WIDTH_MAJOR, area);
                input_report_abs(ts->input, ABS_MT_POSITION_X, x);
                input_report_abs(ts->input, ABS_MT_POSITION_Y, y);
			} else {
				ts->prev_Area[touchID] = 0;
				input_report_abs(ts->input, ABS_MT_TRACKING_ID, -1);
			}	
		}

	}

	// touch key
	if (rd.sfknum)
	{
		sfkNum = rd.sfknum;
		for (i=0;i<sfkNum;i++)
		{
			sfkID = rd.trk[i].sfk_id_num; 
			sfkUpDn= rd.trk[i].sfkrnd; 

			for(keycnt = 0; keycnt < ts->platform_data->nkeys; keycnt++)
			{
				struct g2tsp_keys_button *keys = &ts->platform_data->keys[keycnt];
				
				if (sfkID == keys->glue_id)	{
					input_report_key(ts->input, keys->code, sfkUpDn);	
				}
			}
		}
	}
 
    input_sync(ts->input);

error_worker:
	return;
}


static irqreturn_t g2tsp_irq_func(int irq, void *handle)
{
    struct g2tsp *ts = (struct g2tsp *)handle;
	disable_irq_nosync(irq);
#if 0	
	udelay(20);
	
	gpio = gpio_get_value(ts->platform_data->irq_gpio);
	if(gpio) 
	{
		printk("  is not tch data !! = %d \r\n", gpio);
		enable_irq(irq);
		return;
	}
#endif
    g2tsp_input_worker(ts);
	enable_irq(irq);
    return IRQ_HANDLED;
}


#ifdef CONFIG_HAS_EARLYSUSPEND
static void g2tsp_suspend_register(struct g2tsp *ts)
{
	int ret;
    int i;

	g2debug("%s\n", __func__);
    for (i=0;i<G2TSP_SUSPEND_REGS;i++)
    {
		ret = ts->bus_ops->write_blk(ts->bus_ops, g2tsp_suspend_reg[i][0], 1, (u8 *)&g2tsp_suspend_reg[i][1]);
        g2debug("%s : write [0x%02x]  0x%02x\n", __func__, 
				g2tsp_suspend_reg[i][0], 
				g2tsp_suspend_reg[i][1]);
    }
}

static void g2tsp_resume_register(struct g2tsp *ts)
{
	int ret;
    int i;

	g2debug("%s\n", __func__);
    for (i=0;i<G2TSP_RESUME_REGS;i++)
    {
		ret = ts->bus_ops->write_blk(ts->bus_ops, g2tsp_resume_reg[i][0], 1, (u8 *)&g2tsp_resume_reg[i][1]);
        g2debug("%s : write [0x%02x]  0x%02x\n", __func__, 
				g2tsp_resume_reg[i][0], 
				g2tsp_resume_reg[i][1]);
    }
}


static void g2tsp_early_suspend(struct early_suspend *h)
{
    struct g2tsp *ts = container_of(h, struct g2tsp, early_suspend);

	g2debug("%s\n", __func__);
    mutex_lock(&ts->mutex);

    disable_irq_nosync(ts->irq);
	ts->platform_data->suspend();
	g2tsp_suspend_register(ts);
    ts->suspend = 1;
    cancel_work_sync(&ts->work);

    mutex_unlock(&ts->mutex);
}

static void g2tsp_late_resume(struct early_suspend *h)
{
    struct g2tsp *ts = container_of(h, struct g2tsp, early_suspend);

	g2debug("%s\n", __func__);
    mutex_lock(&ts->mutex);

	ts->platform_data->wakeup();
	g2tsp_resume_register(ts);
    ts->suspend = 0;
    enable_irq(ts->irq);

    mutex_unlock(&ts->mutex);
}
#endif

static void g2tsp_init_register(struct g2tsp *ts)
{
	int ret;
	int i;

	g2debug("%s\n", __func__);
	//mdelay(30);
    for (i=0;i<G2TSP_INIT_REGS;i++)
    {
		ret = ts->bus_ops->write_blk(ts->bus_ops, g2tsp_init_reg[i][0], 1, (u8*)&g2tsp_init_reg[i][1]);
		g2debug("%s : write [0x%02x]  0x%02x\n", __func__, 
				g2tsp_init_reg[i][0], 
				g2tsp_init_reg[i][1]);
		
		mdelay(5);
    }
    mdelay(30);
	
	
}

#define I2C_READ_WORD       0x9010
#define I2C_WRITE_WORD      0x9020
#define I2C_FLASH_ERASE     0x9080
#define I2C_CHIP_INIT       0x9089

struct reg_data{
    unsigned int addr;
    unsigned int data;
};

long g2tsp_ioctl(struct file *file, unsigned int cmd, unsigned long arg)

{
    struct reg_data ioctl_data;
	u8 addr;
    u8 val;

    switch (cmd)
    {
    case I2C_READ_WORD:
        if (copy_from_user(&ioctl_data, (void *)arg, sizeof(ioctl_data)))
            return -EFAULT;

		addr = (u8)ioctl_data.addr;
		current_ts->bus_ops->read(current_ts->bus_ops, addr, &val);
        ioctl_data.data = (unsigned int)val;

        if (copy_to_user((struct reg_data *)arg, &ioctl_data, sizeof(ioctl_data)))
            return -EFAULT;

    	g2debug1("I2C_READ_WORD done.\n");
        break;
    case I2C_WRITE_WORD:
        if (copy_from_user(&ioctl_data, (void *)arg, sizeof(ioctl_data)))
            return -EFAULT;
	
		addr = (u8)ioctl_data.addr;
		val =  (u8)ioctl_data.data;	
		current_ts->bus_ops->write(current_ts->bus_ops, addr, val);
    	g2debug1("I2C_WRITE_WORD done.\n");
        break;
    case I2C_FLASH_ERASE:
        g2tsp_flash_eraseall(current_ts);
    	g2debug1("I2C_FLASH_ERASE done.\n");
		printk("I2C_FLASH_ERASE done.\n");
        break;
    case I2C_CHIP_INIT:
	
       
   		current_ts->bus_ops->write(current_ts->bus_ops, 0x01, 0x0);
        g2tsp_init_register(current_ts);

    	printk("I2C_CHIP_INIT done.\n");
        break;
    default:
    	g2debug1("unknown ioctl: %x\n", cmd);
        return -ENOIOCTLCMD;
    }
    return 0;
}

static const struct file_operations g2tsp_fileops = {
    .owner   = THIS_MODULE,
    .unlocked_ioctl   = g2tsp_ioctl,
};

static struct miscdevice g2tsp_misc_device = {
    .minor      = MISC_DYNAMIC_MINOR,
    .name       = "s3c-g2touch",
    .fops       = &g2tsp_fileops,
};


static void fw_recovery_work(struct work_struct *work)
{	
	
	printk("[G2TSP] : %s() Enter + \r\n", __func__);
	
	if (current_ts->firmware_download)
	{
		printk("[G2TSP] : Emergency Firmware Update !! \r\n");
		current_ts->current_firmware_version[0] = 0;
		current_ts->current_firmware_version[1] = 0;
		request_firmware_nowait(THIS_MODULE,
		                      FW_ACTION_HOTPLUG,
	                    	  G2TSP_FW_NAME,
	                	      current_ts->pdev,
	            	          GFP_KERNEL,
	        	              current_ts,
	    	                  firmware_request_handler);
		
		current_ts->firmware_download = 0;

	}
}

void *g2tsp_init(struct g2tsp_bus_ops *bus_ops, struct device *pdev)
{
    struct input_dev *input_device;
    struct g2tsp *ts;
    int retval = 0;
	int keycnt;

	g2debug("%s\n", __func__);
    ts = kzalloc(sizeof(*ts), GFP_KERNEL);
    if (ts == NULL) {
        printk(KERN_ERR "%s: Error, kzalloc\n", __func__);
        goto error_alloc_data_failed;
    }

    mutex_init(&ts->mutex);
    ts->pdev = pdev;
    ts->platform_data = pdev->platform_data;
    ts->bus_ops = bus_ops;

    if (ts->platform_data->power)
        retval = ts->platform_data->power(1);
    if (retval) {
        printk(KERN_ERR "%s: platform power control failed! \n", __func__);
        goto error_init;
    }

    /* Create the input device and register it. */
    input_device = input_allocate_device();
    if (!input_device) {
        retval = -ENOMEM;
        printk(KERN_ERR "%s: Error, failed to allocate input device\n",
            __func__);
        goto error_input_allocate_device;
    }

    ts->input = input_device;
    input_device->name = ts->platform_data->name;
    input_device->dev.parent = ts->pdev;

    input_device->phys    = "odroida4-ts/input0";
    //input_device->phys  = "cyttsp4_mt";
    input_device->id.bustype  = BUS_HOST;
    input_device->id.vendor   = 0x16B4;
    input_device->id.product  = 0x0702;
    input_device->id.version  = 0x0001;

    /* enable interrupts */
    ts->irq = gpio_to_irq(ts->platform_data->irq_gpio);
#if 1	
    retval = request_threaded_irq(ts->irq, NULL, 
		g2tsp_irq_func, 
		IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
        ts->input->name, ts);
#else
	
	retval = request_irq(ts->irq,  g2tsp_irq_func, ts->platform_data->irqmode, ts->input->name, ts);

#endif

    if (retval < 0) {
        printk(KERN_ERR "%s: IRQ request failed r=%d\n",
            __func__, retval);
        goto error_set_irq;
    }

    set_bit(EV_SYN, input_device->evbit);
    set_bit(EV_KEY, input_device->evbit);
#ifdef CONFIG_LEDS_CLASS
    set_bit(EV_LED, input_device->evbit);
    set_bit(LED_MISC, input_device->ledbit);
#endif
    set_bit(EV_ABS, input_device->evbit);
    set_bit(INPUT_PROP_DIRECT, input_device->propbit);

	for(keycnt = 0; keycnt < ts->platform_data->nkeys; keycnt++)
	{
		struct g2tsp_keys_button *keys = &ts->platform_data->keys[keycnt];
        set_bit(keys->code & KEY_MAX, input_device->keybit);
	}

	memset(ts->prev_Area, 0, sizeof(ts->prev_Area));

    input_set_abs_params(input_device, ABS_MT_POSITION_X, 0, ts->platform_data->res_x, 0, 0);
    input_set_abs_params(input_device, ABS_MT_POSITION_Y, 0, ts->platform_data->res_y, 0, 0);
    input_set_abs_params(input_device, ABS_MT_TOUCH_MAJOR, 1, 255, 0, 0);
    input_set_abs_params(input_device, ABS_MT_WIDTH_MAJOR, 1, 255, 0, 0);
    input_set_abs_params(input_device, ABS_MT_TRACKING_ID, 1, DEF_TRACKING_ID, 0, 0);
	input_mt_init_slots(input_device, DEF_TRACKING_ID);

	// options check
	ts->x_rev = 0;
	ts->y_rev = 0;
 	ts->firmware_download = 0;

	if (ts->platform_data->options & G2_XREV)
		ts->x_rev = 1;
	
	if (ts->platform_data->options & G2_YREV)
		ts->y_rev = 1;

	if (ts->platform_data->options & G2_FWDOWN)
		ts->firmware_download = 1;

    retval = input_register_device(input_device);
    if (retval) {
        printk(KERN_ERR "%s: Error, failed to register input device\n", __func__);
        goto error_input_register_device;
    }

#ifdef CONFIG_HAS_EARLYSUSPEND
    ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
    ts->early_suspend.suspend = g2tsp_early_suspend;
    ts->early_suspend.resume = g2tsp_late_resume;
    register_early_suspend(&ts->early_suspend);
#endif

    dev_set_drvdata(pdev, ts);

//	ts->platform_data->init();
//	ts->platform_data->reset();

	mdelay(10);
	g2tsp_init_register(ts);
	
	current_ts = ts;

	INIT_DELAYED_WORK_DEFERRABLE(&fw_work, fw_recovery_work);
	schedule_delayed_work(&fw_work, FW_RECOVER_DELAY);

	retval = misc_register(&g2tsp_misc_device);
    if (retval) {
        printk(KERN_ERR "%s: s3c-g2touch misc err\n", __func__);
		goto error_misc_register_device;
    }

#ifdef CONFIG_LEDS_CLASS
	ts->leds.name = TOUCHKEY_BACKLIGHT;
	ts->leds.brightness = LED_FULL;
	ts->leds.max_brightness = LED_FULL;
	ts->leds.brightness_set = msm_tkey_led_set;

	retval = led_classdev_register(pdev, &ts->leds);
	if (retval) {
		printk(KERN_ERR "%s: Failed to register led\n", __func__);
		goto fail_led_reg;
	}
#endif
	printk("G2TSP : %s() Exit !! \r\n", __func__);
    return ts;

#ifdef CONFIG_LEDS_CLASS
fail_led_reg:
	led_classdev_unregister(&ts->leds);
#endif

error_misc_register_device:
	
error_input_register_device:
#ifdef CONFIG_HAS_EARLYSUSPEND
    unregister_early_suspend(&ts->early_suspend);
#endif

    input_unregister_device(input_device);

    if (ts->irq >= 0)
        free_irq(ts->irq, ts);

error_set_irq:
  input_free_device(input_device);
error_input_allocate_device:
    if (ts->platform_data->power)
        ts->platform_data->power(0);
error_init:
    kfree(ts);
error_alloc_data_failed:
    return NULL;
}

void g2tsp_release(void *handle)
{
    struct g2tsp *ts = handle;

	g2debug("%s\n", __func__);
#ifdef CONFIG_HAS_EARLYSUSPEND
    unregister_early_suspend(&ts->early_suspend);
#endif

    cancel_work_sync(&ts->work);
    free_irq(ts->irq, ts);
    input_unregister_device(ts->input);
    input_free_device(ts->input);

    if (ts->platform_data->power)
        ts->platform_data->power(0);

    kfree(ts);
}

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("G2touch touchscreen device driver");
MODULE_AUTHOR("G2TOUCH");

