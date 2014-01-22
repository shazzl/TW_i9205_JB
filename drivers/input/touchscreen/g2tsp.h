#ifndef __G2TSP_H__
#define __G2TSP_H__

#include <linux/kernel.h>
#ifdef CONFIG_LEDS_CLASS
#include <linux/leds.h>
#define TOUCHKEY_BACKLIGHT	"button-backlight"
#endif

#define G2TSP_I2C_NAME  "g2tsp"

struct g2tsp_bus_ops {
    s32 (*write_blk)(void *handle, u8 addr, u8 length, const void *values);
    s32 (*read_blk)(void *handle, u8 addr, u8 length, void *values);
    s32 (*write)(void *handle, u8 addr, u8 value);
    s32 (*read)(void *handle, u8 addr, u8 *value);
};

void *g2tsp_init(struct g2tsp_bus_ops *bus_ops, struct device *pdev);
void g2tsp_release(void *handle);






#define G2TSP_NEW_IC	1


#define G2_DEBUG_PRINT
#define G2_DEBUG_PRINT1
#define G2_DEBUG_PRINT2

#ifdef G2_DEBUG_PRINT
	#define g2debug(fmt, ...)                   \
    	do {                                    \
            printk(fmt, ##__VA_ARGS__);     \
    	} while(0) 
#else
	#define g2debug(fmt, ...) do { } while(0)
#endif

#ifdef G2_DEBUG_PRINT1
	#define g2debug1(fmt, ...)                   \
    	do {                                    \
            printk(fmt, ##__VA_ARGS__);     \
    	} while(0) 
#else
	#define g2debug1(fmt, ...) do { } while(0)
#endif

#ifdef G2_DEBUG_PRINT2
	#define g2debug2(fmt, ...)                   \
    	do {                                    \
            printk(fmt, ##__VA_ARGS__);     \
    	} while(0) 
#else
	#define g2debug2(fmt, ...) do { } while(0)
#endif



struct g2tsp {
	struct device *pdev;
    int irq;

	// options
	int x_rev;
	int y_rev;
	int firmware_download;
	int	current_firmware_version[2];

    struct input_dev *input;
    struct work_struct work;
    struct mutex mutex;
    struct early_suspend early_suspend;
    struct g2tsp_platform_data *platform_data;
    struct g2tsp_bus_ops *bus_ops;
	u8 suspend;
	int prev_Area[10];
#ifdef CONFIG_LEDS_CLASS
	struct led_classdev leds;
	bool tkey_led_reserved;
#endif
};


#endif 
