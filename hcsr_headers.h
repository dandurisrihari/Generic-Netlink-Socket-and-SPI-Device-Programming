#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <asm/uaccess.h>
#include <linux/string.h>
#include <linux/device.h>
#include <linux/jiffies.h>
#include <linux/miscdevice.h>
#include <linux/init.h>
#include <linux/moduleparam.h>
#include <linux/string.h>
#include <linux/ktime.h>
#include <linux/hrtimer.h>
#include <linux/delay.h>
#include <asm/div64.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <asm/delay.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/stat.h>
#include <linux/kthread.h>
#include <linux/semaphore.h>




int ultrasonic_speed= 346;    // 346 m/s
static int gpio_num[80];

static int spi_gpio[80];


struct mutex thread_lock;
struct mutex dev_lock;
struct mutex trig_lock;
struct mutex interrupt_lock;
struct mutex device_list_lock;


struct configure{         // structure to store the user input parameters
    int trigPin; // trigPin=10 if trig is connected to IO 10
    int echoPin; 
};

/* per device structure */
struct HCSR_dev {
    char name[20];                    
    int measurement_flag;
    long long unsigned int distance;
  	int trigger_flag;    
  	int expire_time;
  	int data_available;
  	ktime_t ref1;
  	ktime_t ref2;
  	int irq_num;
  	struct task_struct *thread;
  	unsigned int measured_final_distance;
    int data_available_per_measure;
    struct configure conf; 
    int spi_trig;

};
struct HCSR_dev *HCSR_devp;