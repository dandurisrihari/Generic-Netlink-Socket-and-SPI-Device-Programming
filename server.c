#include <linux/version.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/skbuff.h>
#include <linux/netlink.h>
#include <linux/timer.h>
#include <linux/export.h>
#include <net/genetlink.h>
#include <linux/delay.h>


#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/kthread.h>



#include "netlink_headers.h"
#include "hcsr_headers.h"


// bus number to register on 
#define MY_BUS_NUM 1
//static struct spi_device *spi_device;


static unsigned char xfer_tx[2]={0};

struct spidev_data {
    struct spi_device   *spi;
    unsigned char user_pattern[1][8];
};


static struct spidev_data *spidevp;



static struct genl_family genl_test_family;



static int spidev_transfer(unsigned char ch1, unsigned char ch2);
void table_for_commands(int pin_1, int pin_2);
int pins_set(struct Commands_from_usr_2 *f1);
static int distance_work_handler(void *ptr);
void measuring_dis(void);
void print_pattern(struct Commands_from_usr_1 *f1);
static int spidev_open(void *ptr);
static int spidev_release(void);
int spidev_write_thread(void);
void spi_trig_pin(int pin_1);
static int send_measured_distance_to_user(void);



//function is called when user sends the data
static int genl_test_rx_msg(struct sk_buff* skb, struct genl_info* info)
{
    
    struct Commands_from_usr_1 *pat=NULL;
    struct Commands_from_usr_2 *all_pins=NULL;
    struct Commands_from_usr_3 *do_mes=NULL;
    struct Commands_from_usr_1 *pat_temp=NULL;




//checking for different types of data send by user 
    if (info->attrs[ATTR_MSG_1]) {
    pat=(struct Commands_from_usr_1*)nla_data(info->attrs[ATTR_MSG_1]);
    pat_temp=(struct Commands_from_usr_1 *)kmalloc(sizeof(struct Commands_from_usr_1), GFP_KERNEL);

    memcpy(pat_temp,pat,sizeof(struct Commands_from_usr_1));
   //spidev_open((void *)pat);
    kthread_run(spidev_open,(void *)pat_temp,"spi running in background");   //running printing pattern in background 

    if(IS_ERR(HCSR_devp->thread))
        printk(KERN_INFO "Thread can't be created");
    //printk(KERN_INFO "pattern is %d",pat->pattern.user_pattern[0][2]);
    }

    if (info->attrs[ATTR_MSG_2]) {
    all_pins=(struct Commands_from_usr_2*)nla_data(info->attrs[ATTR_MSG_2]);
    pins_set(all_pins);     //setting up hcsr pins
    spi_trig_pin(all_pins->spi_pins.spi_trig);    //setting spi pins 

   // printk(KERN_INFO "spi pins are is %d",all_pins->spi_pins.spi_trig);
    }

    if (info->attrs[ATTR_MSG_3]) {
    do_mes=(struct Commands_from_usr_3*)nla_data(info->attrs[ATTR_MSG_3]);
        //printk(KERN_INFO "do measurement_flag is %d",do_mes->request_measurement);

        if(do_mes->request_measurement==1){    //checking whether user is askingfor measurement
            kthread_run(distance_work_handler,(void *)NULL,"hcsr running in background");

            if(IS_ERR(HCSR_devp->thread))
                printk(KERN_INFO "Thread can't be created");

        }

    }


    // pat=kmalloc(sizeof(struct Commands_from_usr_1), GFP_KERNEL);
    return 0;
   
}


//setting hcsr pins
int pins_set(struct Commands_from_usr_2 *f1){

        if(f1->hcsr_pins.trig < 0 || f1->hcsr_pins.trig > 19){  // checking  if its valid
           return -EINVAL;
        }
 
        if(f1->hcsr_pins.echo < 0 || f1->hcsr_pins.echo > 19 || f1->hcsr_pins.echo ==7 || f1->hcsr_pins.echo== 8) { // checking if echo is valid
           return -EINVAL;
        }
 
      table_for_commands( f1->hcsr_pins.trig, f1->hcsr_pins.echo);  // pin mux table
      return 0;

}

//triggering for hcsr 

void Triggering_fun (void)  // this function triggers the sensor thus causing ultra sound to be transmitted.
  {  
   // mutex_lock_interruptible(&trig_lock);
    HCSR_devp->trigger_flag=1;
    gpio_set_value_cansleep(HCSR_devp->conf.trigPin,0);
    udelay(2);
    gpio_set_value_cansleep(HCSR_devp->conf.trigPin,1);
    udelay(10);
    gpio_set_value_cansleep(HCSR_devp->conf.trigPin,0);
    HCSR_devp->trigger_flag=0;
    // mutex_unlock(&trig_lock);

    
}


void measuring_dis(void)           
{
    int i;
    unsigned long long sum=0;
    unsigned long long max=0;
    unsigned long long min= 100000000;

    HCSR_devp->measurement_flag=0;
    HCSR_devp->data_available=0;

    for (i=0;i<7;i++)
     {  
        Triggering_fun();
        while(1){
            if(HCSR_devp->data_available_per_measure==1)
                break;
            msleep_interruptible(1);
        }
        HCSR_devp->data_available_per_measure=0;
        sum+=HCSR_devp->distance;
        if(HCSR_devp->distance > max)
            max=HCSR_devp->distance;
        if(HCSR_devp->distance<min)
            min=HCSR_devp->distance;
        mdelay(60);
     }
     
     //eliminating outliers
     sum = sum - max;
     sum = sum - min;
     do_div(sum,5);
     //storing measured distance
     HCSR_devp->measured_final_distance=sum;
     //printk(KERN_INFO "distance is from kernel %d \n",HCSR_devp->measured_final_distance);
     HCSR_devp->measurement_flag=1;
        sum=0;
     HCSR_devp->distance=0;
     //setting data available flag 
     HCSR_devp->data_available=1;


    }




//function to send data to user
static int send_measured_distance_to_user(void){
    unsigned int distance=0;
    void *hdr;
    int res, flags = GFP_ATOMIC;
    struct sk_buff* skb = genlmsg_new(NLMSG_DEFAULT_SIZE, flags);


    distance=HCSR_devp->measured_final_distance;
    if (!skb) {
        printk(KERN_ERR "%d: OOM!!", __LINE__);
        return -1;
    }

    hdr = genlmsg_put(skb, 0, 0, &genl_test_family, flags, GENL_TEST_C_MSG);
    if (!hdr) {
        printk(KERN_ERR "%d: Unknown err !", __LINE__);
        goto nlmsg_fail;
    }

    // snprintf(msg, GENL_TEST_ATTR_MSG_MAX, "Hello group %s\n",
    //         genl_test_mcgrp_names[group]);

    res = nla_put(skb, ATTR_MSG_4,sizeof(unsigned int),(void *)&distance);
    if (res) {
        printk(KERN_ERR "%d: err %d ", __LINE__, res);
        goto nlmsg_fail;
    }

    genlmsg_end(skb, hdr);
    genlmsg_multicast(&genl_test_family, skb, 0, MCGRP0, flags);
    return 0;

nlmsg_fail:
    genlmsg_cancel(skb, hdr);
    nlmsg_free(skb);
    return -1;


}






// function which handles all hcsr part

static int distance_work_handler(void *ptr){

    mutex_lock_interruptible (&thread_lock);
    // struct Commands_from_usr_1 *f1=NULL;

    // send_measured_distance_to_user();
 // printk(KERN_INFO "--------hello 1---------");
    // f1=(struct Commands_from_usr_1 *)ptr;

    //printk(KERN_INFO "--------hello 2---------");

   // kthread_run(&spidev_open, (void *)f1,"spidev_write_kthread");
   // printk(KERN_INFO "--------test 4------------\n");

    //printk(KERN_INFO "echo pin is %d",f1->pins.echo);
   // pins_set(f1);

  
//measures distance
    measuring_dis();
    //wait till measurement is done
        while(1){
        if(HCSR_devp->data_available==1){
           // printk(KERN_INFO "Data available");
            break;
        }
        msleep_interruptible(1);
    }

//transfer data to user
    send_measured_distance_to_user();
    gpio_set_value_cansleep(HCSR_devp->conf.trigPin, 0);
    gpio_set_value_cansleep(HCSR_devp->conf.echoPin, 0);

    //printk(KERN_INFO "-------done-----------");




    mutex_unlock(&thread_lock);

    do_exit(0);


}


//function to clear all spidata
static int spidev_release(void)
{
    int status = 0;
    unsigned char i=0;

    spidev_transfer(0x0F, 0x00);
    spidev_transfer(0x0C, 0x01);
    spidev_transfer(0x0B, 0x07);
    spidev_transfer(0x09, 0x00);
    spidev_transfer(0x0A, 0x02);  

    for(i=1; i < 9; i++)
    {
        spidev_transfer(i, 0x00);
    }
    
    //printk("spidev LED driver is released\n");
    return status;
}



//echo trigger handler
static irq_handler_t handling_irq(unsigned int irq, void *dev_id) // interrupt handler
{
    // mutex_lock_interruptible(&interrupt_lock);

    long long int t=0;
    int val=0;
    HCSR_devp->data_available_per_measure=0;
    
    val=gpio_get_value(HCSR_devp->conf.echoPin);
    if(val==1)                                              // rising edge
    {
    HCSR_devp->ref1=ktime_set(0,0);
    HCSR_devp->ref1= ktime_get();
    irq_set_irq_type(HCSR_devp->irq_num,IRQ_TYPE_EDGE_FALLING);
    }

    else                    // falling edge

    {
        
        HCSR_devp->ref2=ktime_set(0,0);             // performing the calculations and fiding the distance.
        HCSR_devp->ref2=ktime_get();
        //ktime_t time_lapsed = ktime_sub(ref2, HCSR_dev->ref1);
        t = ktime_to_us(ktime_sub(HCSR_devp->ref2,HCSR_devp->ref1));
        if(t>=38000){
            printk(KERN_INFO "no object is detected");
            HCSR_devp->distance=0;
            HCSR_devp->expire_time=1;
        } 
        else{
            HCSR_devp->distance = ((t*ultrasonic_speed)/2);
            do_div(HCSR_devp->distance,10000);
           // printk("distance measured %lld \n",HCSR_devp->distance);
        } 

        HCSR_devp->data_available_per_measure=1;
        irq_set_irq_type(HCSR_devp->irq_num,IRQ_TYPE_EDGE_RISING);  //changing edge type tp rising
    }
    // mutex_unlock(&interrupt_lock);
    return (irq_handler_t) IRQ_HANDLED;
    }


//function to print pattern
void print_pattern(struct Commands_from_usr_1 *f1){
    int j=0;
      int  k=0;
      unsigned int row[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
    // struct task_struct *task;
    for(j=0;j<8;j++){
        spidevp->user_pattern[0][j] = f1->pattern.user_pattern[0][j];
    }

    

    while(k<8){
        spidev_transfer(row[k], spidevp->user_pattern[0][k]);
        k++;
    }

}


// Write pattern byte by byte to the SPI device - LED matrix using spi_sync 
static int spidev_transfer(unsigned char ch1, unsigned char ch2)
{
    xfer_tx[0] = ch1;
    xfer_tx[1] = ch2;

    gpio_set_value_cansleep(HCSR_devp->spi_trig, 0);
     spi_write(spidevp->spi, &xfer_tx, sizeof(xfer_tx));
    gpio_set_value_cansleep(HCSR_devp->spi_trig, 1);
    return 1;
}



static int spidev_open(void *ptr)
{ 
    struct Commands_from_usr_1 *f1=NULL;
mutex_lock_interruptible (&device_list_lock); 
   // printk(KERN_INFO "INSIDE SPIDEV OPEN");
     
 f1=(struct Commands_from_usr_1 *)ptr;

//printk(KERN_INFO "pattern is FROM THREAD %d\n",f1->pattern.user_pattern[0][2]);
    while(1){
        if(HCSR_devp->spi_trig>0)
            break;
        msleep_interruptible(1);
    }
    //printk(KERN_INFO "HCSR_devp->spi_trig %d \n",HCSR_devp->spi_trig);

    //clear the pattern before new one comes in 
    spidev_release();


  

    // for(i=1; i < 9; i++)
    // {
    //     spidev_transfer(i, 0x00);
    // }
    print_pattern(f1);


   // HCSR_devp->spi_trig=-1;
   mutex_unlock(&device_list_lock);
    return 0;
}


//multicast groups
static const struct genl_multicast_group genl_test_mcgrps[] = {
    [MCGRP0] = { .name = GENL_TEST_MCGRP0_NAME, },
};



//operations
static const struct genl_ops genl_test_ops[] = {
    {
        .cmd = GENL_TEST_C_MSG,
        .policy = genl_test_policy,
        .doit = genl_test_rx_msg,
        .dumpit = NULL,
    },
};



static struct genl_family genl_test_family = {
    .name = GENL_TEST_FAMILY_NAME,
    .version = 1,
    .maxattr = GENL_TEST_ATTR_MAX,
    .netnsok = false,
    .module = THIS_MODULE,
    .ops = genl_test_ops,
    .n_ops = ARRAY_SIZE(genl_test_ops),
    .mcgrps = genl_test_mcgrps,
    .n_mcgrps = ARRAY_SIZE(genl_test_mcgrps),
};








static int __init genl_test_init(void)
{
    int rc,i,ret;
    struct spi_master *master;

        //Register information about your slave device:
    struct spi_board_info spi_device_info = {
        .modalias = "LED_DOT_MATRIX",
        .max_speed_hz = 500000, //speed your device (slave) can handle
        .bus_num = MY_BUS_NUM,
        .chip_select = 1,
        .mode = 3,
    };

    printk(KERN_INFO "genl_test: initializing netlink\n");

    rc = genl_register_family(&genl_test_family);
    if (rc)
        goto failure;
    //printk(KERN_INFO "------------haii 0---------\n");

    HCSR_devp = (struct HCSR_dev *)kmalloc(sizeof(struct HCSR_dev), GFP_KERNEL);
    if (!HCSR_devp){
        printk("Bad Kmalloc HCSR-DEVP\n"); return -ENOMEM;
    }

    sprintf(HCSR_devp->name,"HCSR");

    HCSR_devp->distance = 0;
    HCSR_devp->measurement_flag=0;
    HCSR_devp->trigger_flag=0;    
    HCSR_devp->expire_time=0;
    HCSR_devp->data_available=0;
    HCSR_devp->irq_num=0;
    HCSR_devp->measured_final_distance=0;
    HCSR_devp->data_available_per_measure=0;

    HCSR_devp->spi_trig=-1;
    // HCSR_devp->thread_count=0;

    for (i = 0; i <80; i++){
        gpio_num[i]=-1;
    }

    for (i= 0; i <80; i++)
    {
        spi_gpio[i]=-1;
    }

   // mutex_init(&dev_lock);
    mutex_init(&thread_lock);

    //mutex_init(&trig_lock);
    //mutex_init(&interrupt_lock);
    mutex_init(&device_list_lock);

///////////////////////////////////////////////////////////////////////////////////////////////////


    spidevp = kzalloc(sizeof(*spidevp), GFP_KERNEL);
    //printk(KERN_INFO "---------nani 1-----------");
    if(!spidevp)
    {
        return -ENOMEM;
    }

     
    //To send data we have to know what spi port/pins should be used. This information 
      //can be found in the device-tree.
   // printk(KERN_INFO "---------nani 2-----------");
    master = spi_busnum_to_master( spi_device_info.bus_num );
    if( !master ){
        printk("MASTER not found.\n");
            return -ENODEV;
    }
     
    // printk(KERN_INFO "---------nani 3-----------");
    // create a new slave device, given the master and device info
    spidevp->spi = spi_new_device( master, &spi_device_info );
 
    if( !spidevp->spi ) {
        printk("FAILED to create slave.\n");
        return -ENODEV;
    }
     
    spidevp->spi->bits_per_word = 8;
    //printk(KERN_INFO "---------nani 4-----------");
    ret = spi_setup( spidevp->spi );
     
    if( ret ){
        printk("FAILED to setup slave.\n");
        spi_unregister_device( spidevp->spi );
        return -ENODEV;
    }
 
 ///////////////////////////////////////////////////////////////////////////////

    gpio_free(44);
    gpio_free(72);
    gpio_free(46);
    gpio_free(24);
   //gpio_free(42);
    gpio_free(30);
   //gpio_free(15);
    //printk(KERN_INFO "------------haii 3---------");
    /* Request the required the GPIOs */
    gpio_request_one(44, GPIOF_DIR_OUT, "MOSI_MUX1");
    gpio_request_one(72, GPIOF_OUT_INIT_LOW, "MOSI_MUX2");
    gpio_request_one(46, GPIOF_DIR_OUT, "SPI_SCK");
    gpio_request_one(24, GPIOF_DIR_OUT, "MOSI_SHIFT");
    //gpio_request_one(42, GPIOF_DIR_OUT, "SS_SHIFT");
    gpio_request_one(30, GPIOF_DIR_OUT, "SCK_SHIFT");
    //gpio_request_one(15, GPIOF_DIR_OUT, "SS_PIN");
    //printk(KERN_INFO "------------haii 4---------\n");
    /* Initiliase GPIO values */
    gpio_set_value_cansleep(44, 1);
    gpio_set_value_cansleep(72, 0);
    gpio_set_value_cansleep(46, 1);
    gpio_set_value_cansleep(24, 0);
  // gpio_set_value_cansleep(42, 0);
    gpio_set_value_cansleep(30, 0);

    /* Initialise the Slave Select as HIGH */
    //gpio_set_value_cansleep(15, 1);



    return 0;
////////////////////////////////////////////////////////////////////////////////////////////////////////

failure:
    printk(KERN_DEBUG "genl_test: error occurred in %s\n", __func__);
    return -EINVAL;
}

module_init(genl_test_init);

static void genl_test_exit(void)
{
    int i;
    genl_unregister_family(&genl_test_family);
    spi_unregister_device( spidevp->spi );
    free_irq(HCSR_devp->irq_num,HCSR_devp);

    gpio_free(44);
    gpio_free(72);
    gpio_free(46);
    gpio_free(24);
    gpio_free(30);

    for (i= 0; i <80; i++){
        if(spi_gpio[i]!=-1)
            gpio_free(spi_gpio[i]);
    }
    for (i= 0; i <80; i++)
    {
        if(gpio_num[i]!=-1)
        gpio_free(gpio_num[i]);
    }
    kfree(HCSR_devp);

}
module_exit(genl_test_exit);
MODULE_LICENSE("GPL");


//function to set spi pins
void spi_trig_pin(int pin_1){

        switch(pin_1){  //trig, output
        case 0:    
            //printk("trig pin is 0\n");
            spi_gpio[11] = 11;
            spi_gpio[32] = 32;
            if( gpio_request(11, "gpio_out_11") != 0 )  printk("gpio_out_11 error!\n");
            if( gpio_request(32, "dir_out_32") != 0 )  printk("dir_out_32 error!\n");
            HCSR_devp->spi_trig = 11;
            //printk("HCSR_devp->spi_trig is:%d \n", HCSR_devp->spi_trig);
            gpio_direction_output(HCSR_devp->spi_trig, 1);
            gpio_direction_output(spi_gpio[32], 0);
            //gpio_set_value(32, 0);
            break;
        case 1:
           // printk("trig pin is 1\n");
            spi_gpio[12] = 12;
            spi_gpio[28] = 28;
            spi_gpio[45] = 45;
            if( gpio_request(12, "gpio_out_12") != 0 )  printk("gpio_out_12 error!\n");
            if( gpio_request(28, "dir_out_28") != 0 )  printk("dir_out_28 error!\n");
            if( gpio_request(45, "pin_mux_45") != 0 )  printk("pin_mux_45 error!\n");
            HCSR_devp->spi_trig = 12;
            //printk("HCSR_devp->spi_trig is:%d \n", HCSR_devp->spi_trig);
            gpio_direction_output(28, 0);
            gpio_direction_output(HCSR_devp->spi_trig, 1);
            //gpio_set_value_cansleep(28, 0);
            gpio_direction_output(45, 0);
            break;
        case 2:
            //printk("trig pin is 2\n");
            spi_gpio[13] = 13;
            spi_gpio[34] = 34;
            spi_gpio[77] = 77;
            if( gpio_request(13, "gpio_out_13") != 0 )  printk("gpio_out_13 error!\n");
            if( gpio_request(34, "dir_out_34") != 0 )  printk("dir_out_34 error!\n");
            if( gpio_request(77, "pin_mux_77") != 0 )  printk("pin_mux_77 error!\n");
            HCSR_devp->spi_trig = 13;
            //printk("HCSR_devp->spi_trig is:%d \n", HCSR_devp->spi_trig);
            gpio_direction_output(HCSR_devp->spi_trig, 1);
            gpio_direction_output(34, 0);
            gpio_direction_output(77, 0);
            break;
        case 3:
            //printk("trig pin is 3\n");
            spi_gpio[14] = 14;
            spi_gpio[16]= 16;
            spi_gpio[76]= 76;
            spi_gpio[64]= 64;
            if( gpio_request(14, "gpio_out_14") != 0 )  printk("gpio_out_14 error!\n");
            if( gpio_request(16, "dir_out_16") != 0 )  printk("dir_out_16 error!\n");
            if( gpio_request(76, "pin_mux_76") != 0 )  printk("pin_mux_76 error!\n");
            if( gpio_request(64, "pin_mux_64") != 0 )  printk("pin_mux_64 error!\n");
            HCSR_devp->spi_trig = 14;
            //printk("HCSR_devp->spi_trig is:%d \n", HCSR_devp->spi_trig);
            gpio_direction_output(HCSR_devp->spi_trig, 1);
            gpio_direction_output(16, 0);
            gpio_direction_output(76, 0);
            gpio_direction_output(64, 0);
            break;
        case 4:
            //printk("trig pin is 4\n");
            spi_gpio[6] = 6;
            spi_gpio[36]= 36;
            if( gpio_request(6, "gpio_out_6") != 0 )  printk("gpio_out_6 error!\n");
            if( gpio_request(36, "dir_out_36") != 0 )  printk("dir_out_36 error!\n");
            HCSR_devp->spi_trig = 6;
            //printk("HCSR_devp->spi_trig is:%d \n", HCSR_devp->spi_trig);
            gpio_direction_output(HCSR_devp->spi_trig, 1);
            gpio_direction_output(36, 0);
            break;
        case 5:
            //printk("trig pin is 5\n");
            spi_gpio[0] = 0;
            spi_gpio[18] = 18;
            spi_gpio[66] = 66;
            if( gpio_request(0, "gpio_out_0") != 0 )  printk("gpio_out_0 error!\n");
            if( gpio_request(18, "dir_out_18") != 0 )  printk("dir_out_18 error!\n");
            if( gpio_request(66, "pin_mux_66") != 0 )  printk("pin_mux_66 error!\n");
            HCSR_devp->spi_trig = 0;
            //printk("HCSR_devp->spi_trig is:%d \n", HCSR_devp->spi_trig);
            gpio_direction_output(18, 0);
            gpio_direction_output(66, 0);
            gpio_direction_output(HCSR_devp->spi_trig, 1);
            break;
        case 6:
            //printk("trig pin is 6\n");
            spi_gpio[1] = 1;
            spi_gpio[20] = 20;
            spi_gpio[68] = 68;
            if( gpio_request(1, "gpio_out_1") != 0 )  printk("gpio_out_1 error!\n");
            if( gpio_request(20, "dir_out_20") != 0 )  printk("dir_out_20 error!\n");
            if( gpio_request(68, "pin_mux_68") != 0 )  printk("pin_mux_68 error!\n");
            HCSR_devp->spi_trig = 1;
            //printk("HCSR_devp->spi_trig is:%d \n", HCSR_devp->spi_trig);
            gpio_direction_output(HCSR_devp->spi_trig, 1);
            gpio_direction_output(20, 0);
            gpio_direction_output(68, 0);
            break;
        case 7:
            //printk("trig pin is 7\n");
            spi_gpio[38] = 38;
            if( gpio_request(38, "gpio_out_38") != 0 )  printk("gpio_out_38 error!\n");
            HCSR_devp->spi_trig = 38;
            //printk("HCSR_devp->spi_trig is:%d \n", HCSR_devp->spi_trig);
            gpio_direction_output(HCSR_devp->spi_trig, 1);
            break;
        case 8:
            //printk("trig pin is 8\n");
            spi_gpio[40] = 40;
            if( gpio_request(40, "gpio_out_40") != 0 )  printk("gpio_out_40 error!\n");
            HCSR_devp->spi_trig = 40;
            //printk("HCSR_devp->spi_trig is:%d \n", HCSR_devp->spi_trig);
            gpio_direction_output(HCSR_devp->spi_trig, 1);
            break;
 
        case 9:
            //printk("trig pin is 9\n");
            spi_gpio[4] = 4;
            spi_gpio[22] = 22;
            spi_gpio[70] = 70;
            if( gpio_request(4, "gpio_out_4") != 0 )  printk("gpio_out_4 error!\n");
            if( gpio_request(22, "dir_out_22") != 0 )  printk("dir_out_22 error!\n");
            if( gpio_request(70, "pin_mux_70") != 0 )  printk("pin_mux_70 error!\n");
            HCSR_devp->spi_trig = 4;
            //printk("HCSR_devp->spi_trig is:%d \n", HCSR_devp->spi_trig);
            gpio_direction_output(HCSR_devp->spi_trig, 1);
            gpio_direction_output(22, 0);
            gpio_direction_output(70, 0);
            break;
        case 10:
            //printk("trig pin is 10\n");
            spi_gpio[10] = 10;
            spi_gpio[26] = 26;
            spi_gpio[74] = 74;
            if( gpio_request(10, "gpio_out_10") != 0 )  printk("gpio_out_10 error!\n");
            if( gpio_request(26, "dir_out_26") != 0 )  printk("dir_out_26 error!\n");
            if( gpio_request(74, "pin_mux_74") != 0 )  printk("pin_mux_74 error!\n");
            HCSR_devp->spi_trig = 10;
            //printk("HCSR_devp->spi_trig is:%d \n", HCSR_devp->spi_trig);
            gpio_direction_output(HCSR_devp->spi_trig, 1);
            gpio_direction_output(26, 0);
            gpio_direction_output(74, 0);
            break;
        case 11:
            //printk("trig pin is 11\n");
            spi_gpio[5] = 5;
            spi_gpio[24] = 24;
            spi_gpio[44] = 44;
            spi_gpio[72] = 72;
            if( gpio_request(5, "gpio_out_5") != 0 )  printk("gpio_out_5 error!\n");
            if( gpio_request(24, "dir_out_24") != 0 )  printk("dir_out_24 error!\n");
            if( gpio_request(44, "pin_mux_44") != 0 )  printk("pin_mux_44 error!\n");
            if( gpio_request(72, "pin_mux_72") != 0 )  printk("pin_mux_72 error!\n");
            HCSR_devp->spi_trig = 5;
            //printk("HCSR_devp->spi_trig is:%d \n", HCSR_devp->spi_trig);
            gpio_direction_output(HCSR_devp->spi_trig, 1);
            gpio_direction_output(24, 0);
            gpio_direction_output(44, 0);
            gpio_direction_output(72, 0);
            break;
        case 12:
            //printk("trig pin is 12\n");
            spi_gpio[15] = 15;
            spi_gpio[42] = 42;
            if( gpio_request(15, "gpio_out_15") != 0 )  printk("gpio_out_15 error!\n");
            if( gpio_request(42, "dir_out_42") != 0 )  printk("dir_out_42 error!\n");
            HCSR_devp->spi_trig = 15;
            //printk("HCSR_devp->spi_trig is:%d \n", HCSR_devp->spi_trig);
            gpio_direction_output(42, 0);
            gpio_direction_output(HCSR_devp->spi_trig, 1);
            break;
        case 13:
            //printk("trig pin is 13\n");
            spi_gpio[7] = 7;
            spi_gpio[30] = 30;
            spi_gpio[46] = 46;
            if( gpio_request(7, "gpio_out_7") != 0 )  printk("gpio_out_7 error!\n");
            if( gpio_request(30, "dir_out_30") != 0 )  printk("dir_out_30 error!\n");
            if( gpio_request(46, "pin_mux_46") != 0 )  printk("pin_mux_46 error!\n");
            HCSR_devp->spi_trig = 7;
            //printk("HCSR_devp->spi_trig is:%d \n", HCSR_devp->spi_trig);
            gpio_direction_output(HCSR_devp->spi_trig, 1);
            gpio_direction_output(30, 0);
            gpio_direction_output(46, 0);
            break;
        case 14:
            //printk("trig pin is 14\n");
            spi_gpio[48] = 48;
            if( gpio_request(48, "gpio_out_48") != 0 )  printk("gpio_out_48 error!\n");
            HCSR_devp->spi_trig = 48;
            //printk("HCSR_devp->spi_trig is:%d \n", HCSR_devp->spi_trig);
            gpio_direction_output(HCSR_devp->spi_trig, 1);
            break;
        case 15:
            //printk("trig pin is 15\n");
            spi_gpio[50] = 50;
            if( gpio_request(50, "gpio_out_50") != 0 )  printk("gpio_out_50 error!\n");
            HCSR_devp->spi_trig = 50;
            //printk("HCSR_devp->spi_trig is:%d \n", HCSR_devp->spi_trig);
            gpio_direction_output(HCSR_devp->spi_trig, 1);
            break;
        case 16:
            //printk("trig pin is 16\n");
            spi_gpio[52] = 52;
            if( gpio_request(52, "gpio_out_52") != 0 )  printk("gpio_out_52 error!\n");
            HCSR_devp->spi_trig = 52;
            //printk("HCSR_devp->spi_trig is:%d \n", HCSR_devp->spi_trig);
            gpio_direction_output(HCSR_devp->spi_trig, 1);
            break;
        case 17:
            //printk("trig pin is 17\n");
            spi_gpio[54] = 54;
            if( gpio_request(54, "gpio_out_54") != 0 )  printk("gpio_out_54 error!\n");
            HCSR_devp->spi_trig = 54;
            //printk("HCSR_devp->spi_trig is:%d \n", HCSR_devp->spi_trig);
            gpio_direction_output(HCSR_devp->spi_trig, 1);
            break;
        case 18:
            //printk("trig pin is 18\n");
            spi_gpio[56] = 56;
            spi_gpio[60] = 60;
            spi_gpio[78] = 78;
            if( gpio_request(56, "gpio_out_56") != 0 )  printk("gpio_out_56 error!\n");
            if( gpio_request(60, "pin_mux_60") != 0 )  printk("pin_mux_60 error!\n");
            if( gpio_request(78, "pin_mux_78") != 0 )  printk("pin_mux_78 error!\n");
            HCSR_devp->spi_trig = 56;
            //printk("HCSR_devp->spi_trig is:%d \n", HCSR_devp->spi_trig);
            gpio_direction_output(HCSR_devp->spi_trig, 1);
            gpio_direction_output(60, 0);
            gpio_direction_output(78, 0);
            break;
        case 19:
            //printk("trig pin is 19\n");
            spi_gpio[58] = 58;
            spi_gpio[60] = 60;
            spi_gpio[79] = 79;
            if( gpio_request(58, "gpio_out_58") != 0 )  printk("gpio_out_58 error!\n");
            if( gpio_request(60, "pin_mux_60") != 0 )  printk("pin_mux_60 error!\n");
            if( gpio_request(79, "pin_mux_79") != 0 )  printk("pin_mux_79 error!\n");
            HCSR_devp->spi_trig = 58;
            //printk("HCSR_devp->spi_trig is:%d \n", HCSR_devp->spi_trig);
            gpio_direction_output(HCSR_devp->spi_trig, 1);
            gpio_direction_output(60, 0);
            gpio_direction_output(79, 0);
            break;
    }

}


//function to set hcsr pins

void table_for_commands(int pin_1, int pin_2){  // 1: output, 2:input
    switch(pin_1){  //trig, output
        case 0:    
            //printk("trig pin is 0\n");
            gpio_num[11] = 11;
            gpio_num[32] = 32;
            if( gpio_request(11, "gpio_out_11") != 0 )  printk("gpio_out_11 error!\n");
            if( gpio_request(32, "dir_out_32") != 0 )  printk("dir_out_32 error!\n");
            HCSR_devp->conf.trigPin = 11;
            //printk("HCSR_devp->conf.trigPin is:%d \n", HCSR_devp->conf.trigPin);
            gpio_direction_output(HCSR_devp->conf.trigPin, 0);
            gpio_direction_output(32, 0);
            break;
        case 1:
           // printk("trig pin is 1\n");
            gpio_num[12] = 12;
            gpio_num[28] = 28;
            gpio_num[45] = 45;
            if( gpio_request(12, "gpio_out_12") != 0 )  printk("gpio_out_12 error!\n");
            if( gpio_request(28, "dir_out_28") != 0 )  printk("dir_out_28 error!\n");
            if( gpio_request(45, "pin_mux_45") != 0 )  printk("pin_mux_45 error!\n");
            HCSR_devp->conf.trigPin = 12;
            //printk("HCSR_devp->conf.trigPin is:%d \n", HCSR_devp->conf.trigPin);
            gpio_direction_output(HCSR_devp->conf.trigPin, 0);
            gpio_direction_output(28, 0);
            gpio_direction_output(45, 0);
            break;
        case 2:
            //printk("trig pin is 2\n");
            gpio_num[13] = 13;
            gpio_num[34] = 34;
            gpio_num[77] = 77;
            if( gpio_request(13, "gpio_out_13") != 0 )  printk("gpio_out_13 error!\n");
            if( gpio_request(34, "dir_out_34") != 0 )  printk("dir_out_34 error!\n");
            if( gpio_request(77, "pin_mux_77") != 0 )  printk("pin_mux_77 error!\n");
            HCSR_devp->conf.trigPin = 13;
            //printk("HCSR_devp->conf.trigPin is:%d \n", HCSR_devp->conf.trigPin);
            gpio_direction_output(HCSR_devp->conf.trigPin, 0);
            gpio_direction_output(34, 0);
            gpio_direction_output(77, 0);
            break;
        case 3:
            //printk("trig pin is 3\n");
            gpio_num[14] = 14;
            gpio_num[16]= 16;
            gpio_num[76]= 76;
            gpio_num[64]= 64;
            if( gpio_request(14, "gpio_out_14") != 0 )  printk("gpio_out_14 error!\n");
            if( gpio_request(16, "dir_out_16") != 0 )  printk("dir_out_16 error!\n");
            if( gpio_request(76, "pin_mux_76") != 0 )  printk("pin_mux_76 error!\n");
            if( gpio_request(64, "pin_mux_64") != 0 )  printk("pin_mux_64 error!\n");
            HCSR_devp->conf.trigPin = 14;
            //printk("HCSR_devp->conf.trigPin is:%d \n", HCSR_devp->conf.trigPin);
            gpio_direction_output(HCSR_devp->conf.trigPin, 0);
            gpio_direction_output(16, 0);
            gpio_direction_output(76, 0);
            gpio_direction_output(64, 0);
            break;
        case 4:
            //printk("trig pin is 4\n");
            gpio_num[6] = 6;
            gpio_num[36]= 36;
            if( gpio_request(6, "gpio_out_6") != 0 )  printk("gpio_out_6 error!\n");
            if( gpio_request(36, "dir_out_36") != 0 )  printk("dir_out_36 error!\n");
            HCSR_devp->conf.trigPin = 6;
            //printk("HCSR_devp->conf.trigPin is:%d \n", HCSR_devp->conf.trigPin);
            gpio_direction_output(HCSR_devp->conf.trigPin, 0);
            gpio_direction_output(36, 0);
            break;
        case 5:
            //printk("trig pin is 5\n");
            gpio_num[0] = 0;
            gpio_num[18] = 18;
            gpio_num[66] = 66;
            if( gpio_request(0, "gpio_out_0") != 0 )  printk("gpio_out_0 error!\n");
            if( gpio_request(18, "dir_out_18") != 0 )  printk("dir_out_18 error!\n");
            if( gpio_request(66, "pin_mux_66") != 0 )  printk("pin_mux_66 error!\n");
            HCSR_devp->conf.trigPin = 0;
            //printk("HCSR_devp->conf.trigPin is:%d \n", HCSR_devp->conf.trigPin);
            gpio_direction_output(HCSR_devp->conf.trigPin, 0);
            gpio_direction_output(18, 0);
            gpio_direction_output(66, 0);
            break;
        case 6:
            //printk("trig pin is 6\n");
            gpio_num[1] = 1;
            gpio_num[20] = 20;
            gpio_num[68] = 68;
            if( gpio_request(1, "gpio_out_1") != 0 )  printk("gpio_out_1 error!\n");
            if( gpio_request(20, "dir_out_20") != 0 )  printk("dir_out_20 error!\n");
            if( gpio_request(68, "pin_mux_68") != 0 )  printk("pin_mux_68 error!\n");
            HCSR_devp->conf.trigPin = 1;
            //printk("HCSR_devp->conf.trigPin is:%d \n", HCSR_devp->conf.trigPin);
            gpio_direction_output(HCSR_devp->conf.trigPin, 0);
            gpio_direction_output(20, 0);
            gpio_direction_output(68, 0);
            break;
        case 7:
            //printk("trig pin is 7\n");
            gpio_num[38] = 38;
            if( gpio_request(38, "gpio_out_38") != 0 )  printk("gpio_out_38 error!\n");
            HCSR_devp->conf.trigPin = 38;
            //printk("HCSR_devp->conf.trigPin is:%d \n", HCSR_devp->conf.trigPin);
            gpio_direction_output(HCSR_devp->conf.trigPin, 0);
            break;
        case 8:
            //printk("trig pin is 8\n");
            gpio_num[40] = 40;
            if( gpio_request(40, "gpio_out_40") != 0 )  printk("gpio_out_40 error!\n");
            HCSR_devp->conf.trigPin = 40;
            //printk("HCSR_devp->conf.trigPin is:%d \n", HCSR_devp->conf.trigPin);
            gpio_direction_output(HCSR_devp->conf.trigPin, 0);
            break;
 
        case 9:
            //printk("trig pin is 9\n");
            gpio_num[4] = 4;
            gpio_num[22] = 22;
            gpio_num[70] = 70;
            if( gpio_request(4, "gpio_out_4") != 0 )  printk("gpio_out_4 error!\n");
            if( gpio_request(22, "dir_out_22") != 0 )  printk("dir_out_22 error!\n");
            if( gpio_request(70, "pin_mux_70") != 0 )  printk("pin_mux_70 error!\n");
            HCSR_devp->conf.trigPin = 4;
            //printk("HCSR_devp->conf.trigPin is:%d \n", HCSR_devp->conf.trigPin);
            gpio_direction_output(HCSR_devp->conf.trigPin, 0);
            gpio_direction_output(22, 0);
            gpio_direction_output(70, 0);
            break;
        case 10:
            //printk("trig pin is 10\n");
            gpio_num[10] = 10;
            gpio_num[26] = 26;
            gpio_num[74] = 74;
            if( gpio_request(10, "gpio_out_10") != 0 )  printk("gpio_out_10 error!\n");
            if( gpio_request(26, "dir_out_26") != 0 )  printk("dir_out_26 error!\n");
            if( gpio_request(74, "pin_mux_74") != 0 )  printk("pin_mux_74 error!\n");
            HCSR_devp->conf.trigPin = 10;
            //printk("HCSR_devp->conf.trigPin is:%d \n", HCSR_devp->conf.trigPin);
            gpio_direction_output(HCSR_devp->conf.trigPin, 0);
            gpio_direction_output(26, 0);
            gpio_direction_output(74, 0);
            break;
        case 11:
            //printk("trig pin is 11\n");
            gpio_num[5] = 5;
            gpio_num[24] = 24;
            gpio_num[44] = 44;
            gpio_num[72] = 72;
            if( gpio_request(5, "gpio_out_5") != 0 )  printk("gpio_out_5 error!\n");
            if( gpio_request(24, "dir_out_24") != 0 )  printk("dir_out_24 error!\n");
            if( gpio_request(44, "pin_mux_44") != 0 )  printk("pin_mux_44 error!\n");
            if( gpio_request(72, "pin_mux_72") != 0 )  printk("pin_mux_72 error!\n");
            HCSR_devp->conf.trigPin = 5;
            //printk("HCSR_devp->conf.trigPin is:%d \n", HCSR_devp->conf.trigPin);
            gpio_direction_output(HCSR_devp->conf.trigPin, 0);
            gpio_direction_output(24, 0);
            gpio_direction_output(44, 0);
            gpio_direction_output(72, 0);
            break;
        case 12:
            //printk("trig pin is 12\n");
            gpio_num[15] = 15;
            gpio_num[42] = 42;
            if( gpio_request(15, "gpio_out_15") != 0 )  printk("gpio_out_15 error!\n");
            if( gpio_request(42, "dir_out_42") != 0 )  printk("dir_out_42 error!\n");
            HCSR_devp->conf.trigPin = 15;
            //printk("HCSR_devp->conf.trigPin is:%d \n", HCSR_devp->conf.trigPin);
            gpio_direction_output(HCSR_devp->conf.trigPin, 0);
            gpio_direction_output(42, 0);
            break;
        case 13:
            //printk("trig pin is 13\n");
            gpio_num[7] = 7;
            gpio_num[30] = 30;
            gpio_num[46] = 46;
            if( gpio_request(7, "gpio_out_7") != 0 )  printk("gpio_out_7 error!\n");
            if( gpio_request(30, "dir_out_30") != 0 )  printk("dir_out_30 error!\n");
            if( gpio_request(46, "pin_mux_46") != 0 )  printk("pin_mux_46 error!\n");
            HCSR_devp->conf.trigPin = 7;
            //printk("HCSR_devp->conf.trigPin is:%d \n", HCSR_devp->conf.trigPin);
            gpio_direction_output(HCSR_devp->conf.trigPin, 0);
            gpio_direction_output(30, 0);
            gpio_direction_output(46, 0);
            break;
        case 14:
            //printk("trig pin is 14\n");
            gpio_num[48] = 48;
            if( gpio_request(48, "gpio_out_48") != 0 )  printk("gpio_out_48 error!\n");
            HCSR_devp->conf.trigPin = 48;
            //printk("HCSR_devp->conf.trigPin is:%d \n", HCSR_devp->conf.trigPin);
            gpio_direction_output(HCSR_devp->conf.trigPin, 0);
            break;
        case 15:
            //printk("trig pin is 15\n");
            gpio_num[50] = 50;
            if( gpio_request(50, "gpio_out_50") != 0 )  printk("gpio_out_50 error!\n");
            HCSR_devp->conf.trigPin = 50;
            //printk("HCSR_devp->conf.trigPin is:%d \n", HCSR_devp->conf.trigPin);
            gpio_direction_output(HCSR_devp->conf.trigPin, 0);
            break;
        case 16:
            //printk("trig pin is 16\n");
            gpio_num[52] = 52;
            if( gpio_request(52, "gpio_out_52") != 0 )  printk("gpio_out_52 error!\n");
            HCSR_devp->conf.trigPin = 52;
            //printk("HCSR_devp->conf.trigPin is:%d \n", HCSR_devp->conf.trigPin);
            gpio_direction_output(HCSR_devp->conf.trigPin, 0);
            break;
        case 17:
            //printk("trig pin is 17\n");
            gpio_num[54] = 54;
            if( gpio_request(54, "gpio_out_54") != 0 )  printk("gpio_out_54 error!\n");
            HCSR_devp->conf.trigPin = 54;
            //printk("HCSR_devp->conf.trigPin is:%d \n", HCSR_devp->conf.trigPin);
            gpio_direction_output(HCSR_devp->conf.trigPin, 0);
            break;
        case 18:
            //printk("trig pin is 18\n");
            gpio_num[56] = 56;
            gpio_num[60] = 60;
            gpio_num[78] = 78;
            if( gpio_request(56, "gpio_out_56") != 0 )  printk("gpio_out_56 error!\n");
            if( gpio_request(60, "pin_mux_60") != 0 )  printk("pin_mux_60 error!\n");
            if( gpio_request(78, "pin_mux_78") != 0 )  printk("pin_mux_78 error!\n");
            HCSR_devp->conf.trigPin = 56;
            //printk("HCSR_devp->conf.trigPin is:%d \n", HCSR_devp->conf.trigPin);
            gpio_direction_output(HCSR_devp->conf.trigPin, 0);
            gpio_direction_output(60, 1);
            gpio_direction_output(78, 1);
            break;
        case 19:
            //printk("trig pin is 19\n");
            gpio_num[58] = 58;
            gpio_num[60] = 60;
            gpio_num[79] = 79;
            if( gpio_request(58, "gpio_out_58") != 0 )  printk("gpio_out_58 error!\n");
            if( gpio_request(60, "pin_mux_60") != 0 )  printk("pin_mux_60 error!\n");
            if( gpio_request(79, "pin_mux_79") != 0 )  printk("pin_mux_79 error!\n");
            HCSR_devp->conf.trigPin = 58;
            //printk("HCSR_devp->conf.trigPin is:%d \n", HCSR_devp->conf.trigPin);
            gpio_direction_output(HCSR_devp->conf.trigPin, 0);
            gpio_direction_output(60, 1);
            gpio_direction_output(79, 1);
            break;
    }
 
    switch(pin_2){  //echo, input
        int ret;
        case 0:    
            gpio_num[11] = 11;
            gpio_num[32] = 32;
            if( gpio_request(11, "gpio_in_11") != 0 )  printk("gpio_in_11 error!\n");
            if( gpio_request(32, "dir_in_32") != 0 )  printk("dir_in_32 error!\n");
            HCSR_devp->irq_num = gpio_to_irq(gpio_num[11]);
            if(HCSR_devp->irq_num<0)  printk("IRQ NUMBER ERROR\n");
            ret = request_irq(HCSR_devp->irq_num,(irq_handler_t) handling_irq, IRQF_TRIGGER_RISING, "rise",(void*)HCSR_devp);
            if (ret < 0)   printk("Error in request_irq\n");
            HCSR_devp->conf.echoPin = 11;
            gpio_direction_input(HCSR_devp->conf.echoPin);
            gpio_set_value_cansleep(32,1);
            break;
        case 1:
            gpio_num[12] = 12;
            gpio_num[28] = 28;
            gpio_num[45] = 45;
            if( gpio_request(12, "gpio_in_12") != 0 )  printk("gpio_in_12 error!\n");
            if( gpio_request(28, "dir_in_28") != 0 )  printk("dir_in_28 error!\n");
            if( gpio_request(45, "pin_mux_45") != 0 )  printk("pin_mux_45 error!\n");
            HCSR_devp->irq_num = gpio_to_irq(gpio_num[12]);
            if(HCSR_devp->irq_num<0)  printk("IRQ NUMBER ERROR\n");
            ret = request_irq(HCSR_devp->irq_num,(irq_handler_t) handling_irq, IRQF_TRIGGER_RISING, "rise", (void*)HCSR_devp);
            if (ret < 0)   printk("Error in request_irq\n");
            HCSR_devp->conf.echoPin = 12;
            gpio_direction_input(HCSR_devp->conf.echoPin);
            gpio_set_value_cansleep(28,1);
            gpio_set_value_cansleep(45, 0);
            break;
        case 2:
            gpio_num[13] = 13;
            gpio_num[34] = 34;
            gpio_num[77] = 77;
            if( gpio_request(13, "gpio_in_13") != 0 )  printk("gpio_in_13 error!\n");
            if( gpio_request(34, "dir_in_34") != 0 )  printk("dir_in_34 error!\n");
            if( gpio_request(77, "pin_mux_77") != 0 )  printk("pin_mux_77 error!\n");
            HCSR_devp->irq_num = gpio_to_irq(gpio_num[13]);
            if(HCSR_devp->irq_num<0)  printk("IRQ NUMBER ERROR\n");
            ret = request_irq(HCSR_devp->irq_num,(irq_handler_t) handling_irq, IRQF_TRIGGER_RISING, "rise", (void*)HCSR_devp);
            if (ret < 0)   printk("Error in request_irq\n");
            HCSR_devp->conf.echoPin = 13;
            gpio_direction_input(HCSR_devp->conf.echoPin);
            gpio_set_value_cansleep(34,1);
            gpio_set_value_cansleep(77, 0);
            break;
        case 3:
            gpio_num[14] = 14;
            gpio_num[16] = 16;
            gpio_num[76] = 76;
            gpio_num[64] = 64;
            if( gpio_request(14, "gpio_in_14") != 0 )  printk("gpio_in_14 error!\n");
            if( gpio_request(16, "dir_in_16") != 0 )  printk("dir_in_16 error!\n");
            if( gpio_request(76, "pin_mux_76") != 0 )  printk("pin_mux_76 error!\n");
            if( gpio_request(64, "pin_mux_64") != 0 )  printk("pin_mux_64 error!\n");
            HCSR_devp->irq_num = gpio_to_irq(gpio_num[14]);
            if(HCSR_devp->irq_num<0)  printk("IRQ NUMBER ERROR\n");
            ret = request_irq(HCSR_devp->irq_num, (irq_handler_t) handling_irq, IRQF_TRIGGER_RISING, "rise", (void*)HCSR_devp);
            if (ret < 0)   printk("Error in request_irq\n");
            HCSR_devp->conf.echoPin = 14;
            gpio_direction_input(HCSR_devp->conf.echoPin);
            gpio_set_value_cansleep(16,1);
            gpio_set_value_cansleep(76, 0);
            gpio_set_value_cansleep(64, 0);
            break;
        case 4:
            gpio_num[6] = 6;
            if( gpio_request(6, "gpio_in_6") != 0 )  printk("gpio_in_6 error!\n");
            HCSR_devp->irq_num= gpio_to_irq(gpio_num[6]);
            if(HCSR_devp->irq_num<0)
            {printk("IRQ NUMBER ERROR\n");}
            ret = request_irq(HCSR_devp->irq_num,(irq_handler_t) handling_irq, IRQF_TRIGGER_RISING, "rise", (void*)HCSR_devp);
            if (ret < 0) 
            {printk("Error in request_irq\n");}
            HCSR_devp->conf.echoPin = 6;
            gpio_direction_input(HCSR_devp->conf.echoPin);

            break;
        case 5:
            gpio_num[0] = 0;
            gpio_num[18] = 18;
            gpio_num[66] = 66;
            if( gpio_request(0, "gpio_in_0") != 0 )  printk("gpio_in_0 error!\n");
            if( gpio_request(18, "dir_in_18") != 0 )  printk("dir_in_18 error!\n");
            if( gpio_request(66, "pin_mux_66") != 0 )  printk("pin_mux_66 error!\n");
            HCSR_devp->irq_num = gpio_to_irq(gpio_num[0]);
            if(HCSR_devp->irq_num<0)  printk("IRQ NUMBER ERROR\n");
            ret = request_irq(HCSR_devp->irq_num,(irq_handler_t) handling_irq, IRQF_TRIGGER_RISING, "rise", (void*)HCSR_devp);
            if (ret < 0)   printk("Error in request_irq\n");
            HCSR_devp->conf.echoPin = 0;
            gpio_direction_input(HCSR_devp->conf.echoPin);
            gpio_set_value_cansleep(18, 1);
            gpio_set_value_cansleep(66, 0);
            break;
        case 6:
            gpio_num[1] = 1;
            gpio_num[20] = 20;
            gpio_num[68] = 68;
            if( gpio_request(1, "gpio_in_1") != 0 )  printk("gpio_in_1 error!\n");
            if( gpio_request(20, "dir_in_20") != 0 )  printk("dir_in_20 error!\n");
            if( gpio_request(68, "pin_mux_68") != 0 )  printk("pin_mux_68 error!\n");
            HCSR_devp->irq_num = gpio_to_irq(gpio_num[1]);
            if(HCSR_devp->irq_num<0)  printk("IRQ NUMBER ERROR\n");
            ret = request_irq(HCSR_devp->irq_num,(irq_handler_t) handling_irq, IRQF_TRIGGER_RISING, "rise", (void*)HCSR_devp);
            if (ret < 0)   printk("Error in request_irq\n");
            HCSR_devp->conf.echoPin = 1;
            gpio_direction_input(HCSR_devp->conf.echoPin);
            gpio_set_value_cansleep(20,1);
            gpio_set_value_cansleep(68, 0);
            break;
        case 7:
            gpio_num[38] = 38;
            if( gpio_request(38, "gpio_in_38") != 0 )  printk("gpio_in_38 error!\n");
            HCSR_devp->irq_num = gpio_to_irq(gpio_num[38]);
            if(HCSR_devp->irq_num<0)  printk("IRQ NUMBER ERROR\n");
            ret = request_irq(HCSR_devp->irq_num,(irq_handler_t) handling_irq, IRQF_TRIGGER_RISING, "rise", (void*)HCSR_devp);
            if (ret < 0)   printk("Error in request_irq\n");
            HCSR_devp->conf.echoPin = 38;
            gpio_direction_input(HCSR_devp->conf.echoPin);
            break;
        case 8:
            gpio_num[40] = 40;
            if( gpio_request(40, "gpio_in_40") != 0 )  printk("gpio_in_40 error!\n");
            HCSR_devp->irq_num = gpio_to_irq(gpio_num[40]);
            if(HCSR_devp->irq_num<0)  printk("IRQ NUMBER ERROR\n");
            ret = request_irq(HCSR_devp->irq_num,(irq_handler_t) handling_irq, IRQF_TRIGGER_RISING, "rise", (void*)HCSR_devp);
            if (ret < 0)   printk("Error in request_irq\n");
            gpio_direction_input(HCSR_devp->conf.echoPin);
            HCSR_devp->conf.echoPin = 40;
            break;
        case 9:
            gpio_num[4] = 4;
            gpio_num[22] = 22;
            gpio_num[70] = 70;
            if( gpio_request(4, "gpio_in_4") != 0 )  printk("gpio_in_4 error!\n");
            if( gpio_request(22, "dir_in_22") != 0 )  printk("dir_in_22 error!\n");
            if( gpio_request(70, "pin_mux_70") != 0 )  printk("pin_mux_70 error!\n");
            HCSR_devp->irq_num = gpio_to_irq(gpio_num[4]);
            if(HCSR_devp->irq_num<0)  printk("IRQ NUMBER ERROR\n");
            ret = request_irq(HCSR_devp->irq_num,(irq_handler_t) handling_irq, IRQF_TRIGGER_RISING, "rise", (void*)HCSR_devp);
            if (ret < 0)   printk("Error in request_irq\n");
            HCSR_devp->conf.echoPin = 4;
            gpio_direction_input(HCSR_devp->conf.echoPin);
            gpio_set_value_cansleep(22,1);
            gpio_set_value_cansleep(70, 0);
            break;
        case 10:
            gpio_num[10] = 10;
            gpio_num[26] = 26;
            gpio_num[74] = 74;
            if( gpio_request(10, "gpio_in_10") != 0 )  printk("gpio_in_10 error!\n");
            if( gpio_request(26, "dir_in_26") != 0 )  printk("dir_in_26 error!\n");
            if( gpio_request(74, "pin_mux_74") != 0 )  printk("pin_mux_74 error!\n");
            HCSR_devp->irq_num = gpio_to_irq(gpio_num[10]);
            if(HCSR_devp->irq_num<0)  printk("IRQ NUMBER ERROR\n");
            ret = request_irq(HCSR_devp->irq_num, (irq_handler_t) handling_irq, IRQF_TRIGGER_RISING, "rise", (void*)HCSR_devp);
            if (ret < 0)   printk("Error in request_irq\n");
            HCSR_devp->conf.echoPin = 10;
            gpio_direction_input(HCSR_devp->conf.echoPin);
            gpio_set_value_cansleep(26,1);
            gpio_set_value_cansleep(74, 0);
            break;
        case 11:
            //printk("echo pin is 11\n");
            gpio_num[5] = 5;
            gpio_num[24] = 24;
            gpio_num[44] = 44;
            gpio_num[72] = 72;
            if( gpio_request(5, "gpio_in_5") != 0 )  printk("gpio_in_5 error!\n");
            if( gpio_request(24, "dir_in_24") != 0 )  printk("dir_in_24 error!\n");
            if( gpio_request(44, "pin_mux_44") != 0 )  printk("pin_mux_44 error!\n");
            if( gpio_request(72, "pin_mux_72") != 0 )  printk("pin_mux_72 error!\n");
            HCSR_devp->irq_num = gpio_to_irq(gpio_num[5]);
            if(HCSR_devp->irq_num<0)  printk("IRQ NUMBER ERROR\n");
            ret = request_irq(HCSR_devp->irq_num,(irq_handler_t) handling_irq, IRQF_TRIGGER_RISING, "rise", (void*)HCSR_devp);
            if (ret < 0)   printk("Error in request_irq\n");
            HCSR_devp->conf.echoPin = 5;
            gpio_direction_input(HCSR_devp->conf.echoPin);
            gpio_set_value_cansleep(24,1);
            gpio_set_value_cansleep(44, 0);
            gpio_set_value_cansleep(72, 0);
            break;
        case 12:
            gpio_num[15] = 15;
            gpio_num[42] = 42;
            if( gpio_request(15, "gpio_in_15") != 0 )  printk("gpio_in_15 error!\n");
            if( gpio_request(42, "dir_in_42") != 0 )  printk("dir_in_42 error!\n");
            HCSR_devp->irq_num = gpio_to_irq(gpio_num[15]);
            if(HCSR_devp->irq_num<0)  printk("IRQ NUMBER ERROR\n");
            ret = request_irq(HCSR_devp->irq_num, (irq_handler_t) handling_irq, IRQF_TRIGGER_RISING, "rise", (void*)HCSR_devp);
            if (ret < 0)   printk("Error in request_irq\n");
            HCSR_devp->conf.echoPin = 15;
            gpio_direction_input(HCSR_devp->conf.echoPin);
            gpio_set_value_cansleep(42,1);
            break;
        case 13:
            gpio_num[7] = 7;
            gpio_num[30] = 30;
            gpio_num[46] = 46;
            if( gpio_request(7, "gpio_in_7") != 0 )  printk("gpio_in_7 error!\n");
            if( gpio_request(30, "dir_in_30") != 0 )  printk("dir_in_30 error!\n");
            if( gpio_request(46, "pin_mux_46") != 0 )  printk("pin_mux_46 error!\n");
            HCSR_devp->irq_num = gpio_to_irq(gpio_num[7]);
            if(HCSR_devp->irq_num<0)  printk("IRQ NUMBER ERROR\n");
            ret = request_irq(HCSR_devp->irq_num,(irq_handler_t) handling_irq, IRQF_TRIGGER_RISING, "rise", (void*)HCSR_devp);
            if (ret < 0)   printk("Error in request_irq\n");
            HCSR_devp->conf.echoPin = 7;
            gpio_direction_input(HCSR_devp->conf.echoPin);
            gpio_set_value_cansleep(30,1 );
            gpio_set_value_cansleep(46, 0);
            break;
        case 14:
            gpio_num[48] = 48;
            if( gpio_request(48, "gpio_in_48") != 0 )  printk("gpio_in_48 error!\n");
            HCSR_devp->irq_num = gpio_to_irq(gpio_num[48]);
            if(HCSR_devp->irq_num<0)  printk("IRQ NUMBER ERROR\n");
            ret = request_irq(HCSR_devp->irq_num, (irq_handler_t) handling_irq, IRQF_TRIGGER_RISING, "rise", (void*)HCSR_devp);
            if (ret < 0)   printk("Error in request_irq\n");
            HCSR_devp->conf.echoPin = 48;
            gpio_direction_input(HCSR_devp->conf.echoPin);
            break;
        case 15:
            gpio_num[50] = 50;
            if( gpio_request(50, "gpio_in_50") != 0 )  printk("gpio_in_50 error!\n");
            HCSR_devp->irq_num = gpio_to_irq(gpio_num[50]);
            if(HCSR_devp->irq_num<0)  printk("IRQ NUMBER ERROR\n");
            ret = request_irq(HCSR_devp->irq_num,(irq_handler_t) handling_irq, IRQF_TRIGGER_RISING, "rise", (void*)HCSR_devp);
            if (ret < 0)   printk("Error in request_irq\n");
            HCSR_devp->conf.echoPin = 50;
            gpio_direction_input(HCSR_devp->conf.echoPin);
            break;
        case 16:
            gpio_num[52] = 52;
            if( gpio_request(52, "gpio_in_52") != 0 )  printk("gpio_in_52 error!\n");
            HCSR_devp->irq_num = gpio_to_irq(gpio_num[52]);
            if(HCSR_devp->irq_num<0)  printk("IRQ NUMBER ERROR\n");
            ret = request_irq(HCSR_devp->irq_num,(irq_handler_t) handling_irq, IRQF_TRIGGER_RISING, "rise", (void*)HCSR_devp);
            if (ret < 0)   printk("Error in request_irq\n");
            HCSR_devp->conf.echoPin = 52;
            gpio_direction_input(HCSR_devp->conf.echoPin);
            break;
        case 17:
            gpio_num[54] = 54;
            if( gpio_request(54, "gpio_in_54") != 0 )  printk("gpio_in_54 error!\n");
            HCSR_devp->irq_num = gpio_to_irq(gpio_num[54]);
            if(HCSR_devp->irq_num<0)  printk("IRQ NUMBER ERROR\n");
            ret = request_irq(HCSR_devp->irq_num,(irq_handler_t) handling_irq, IRQF_TRIGGER_RISING, "rise", (void*)HCSR_devp);
            if (ret < 0)   printk("Error in request_irq\n");
            HCSR_devp->conf.echoPin = 54;
            gpio_direction_input(HCSR_devp->conf.echoPin);
            break;
        case 18:
            gpio_num[56] = 56;
            gpio_num[60] = 60;
            gpio_num[78] = 78;
            if( gpio_request(56, "gpio_in_56") != 0 )  printk("gpio_in_56 error!\n");
            if( gpio_request(60, "pin_mux_60") != 0 )  printk("pin_mux_60 error!\n");
            if( gpio_request(78, "pin_mux_78") != 0 )  printk("pin_mux_78 error!\n");
            HCSR_devp->irq_num = gpio_to_irq(gpio_num[56]);
            if(HCSR_devp->irq_num<0)  printk("IRQ NUMBER ERROR\n");
            ret = request_irq(HCSR_devp->irq_num,(irq_handler_t) handling_irq, IRQF_TRIGGER_RISING, "rise", (void*)HCSR_devp);
            if (ret < 0)   printk("Error in request_irq\n");
            HCSR_devp->conf.echoPin = 56;
            gpio_direction_input(HCSR_devp->conf.echoPin);
            gpio_set_value_cansleep(60, 1);
            gpio_set_value_cansleep(78, 1);
            break;
        case 19:
            gpio_num[58] = 58;
            gpio_num[60] = 60;
            gpio_num[79] = 79;
            if( gpio_request(58, "gpio_in_58") != 0 )  printk("gpio_in_58 error!\n");
            if( gpio_request(60, "pin_mux_60") != 0 )  printk("pin_mux_60 error!\n");
            if( gpio_request(79, "pin_mux_79") != 0 )  printk("pin_mux_79 error!\n");
            HCSR_devp->irq_num = gpio_to_irq(gpio_num[58]);
            if(HCSR_devp->irq_num<0)  printk("IRQ NUMBER ERROR\n");
            ret = request_irq(HCSR_devp->irq_num,(irq_handler_t) handling_irq, IRQF_TRIGGER_RISING, "rise", (void*)HCSR_devp);
            if (ret < 0)   printk("Error in request_irq\n");
            HCSR_devp->conf.echoPin = 58;
            gpio_direction_input(HCSR_devp->conf.echoPin);
            gpio_set_value_cansleep(60, 1);
            gpio_set_value_cansleep(79, 1);
            break;
    }
}