/* G2TOUCH */

#include <linux/input.h>

#ifndef _G2TSP_H_
#define _G2TSP_H_

#define G2_XREV			0x1	
#define G2_YREV			0x2
#define G2_FWDOWN		0x10

struct g2tsp_keys_button {
    unsigned int code; 
    int glue_id;
};


struct g2tsp_platform_data {
	struct g2tsp_keys_button *keys;
	int nkeys;
	u32	res_x;
	u32 res_y;
	u32 options;
	void (*init)(void); 
	void (*suspend)(void); 
	void (*wakeup)(void);
	int (*reset)(void);
	int (*power)(int onoff); 

	u32 gpio_scl;
	u32 gpio_sda;
	void (*i2c_to_gpio)(int enable);

	void (*mt_sync)(struct input_dev *);

    char *name;
    u32 irq_gpio;
	u32 irqmode;

	u32 fw_version[2];
	
};


#endif //_G2TSP_H_
