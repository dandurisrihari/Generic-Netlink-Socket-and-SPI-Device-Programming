#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <ctype.h>
#include <errno.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <netlink/msg.h>
#include <netlink/attr.h>
#include <netlink/genl/genl.h>
#include <netlink/genl/family.h>
#include <netlink/genl/ctrl.h>


#include "netlink_headers.h"

#include <unistd.h>
#include <errno.h>    

#include <time.h>
#include <pthread.h>



#ifndef GRADING
#define MAX7219_CS_PIN 6
#define HCSR04_TRIGGER_PIN 7
#define HCSR04_ECHO_PIN 2
#endif

static char* genl_test_mcgrp_names[GENL_TEST_MCGRP_MAX] = {
	GENL_TEST_MCGRP0_NAME,
	// GENL_TEST_MCGRP1_NAME,
	// GENL_TEST_MCGRP2_NAME,
};


int msleep(long msec);
pthread_mutex_t lock; 
int final_distance;

struct nl_sock* nlsock = NULL;
struct nl_cb *cb = NULL;


long int delay_fun(void);


static int skip_seq_check(struct nl_msg *msg, void *arg)
{
	return NL_OK;
}


//function is called when message is sent from kernel
static int print_rx_msg(struct nl_msg *msg, void* arg)
{
	struct nlattr *attr[GENL_TEST_ATTR_MAX+1];

	genlmsg_parse(nlmsg_hdr(msg), 0, attr, 
			GENL_TEST_ATTR_MAX, genl_test_policy);

	if (!attr[ATTR_MSG_4]) {
		fprintf(stdout, "Kernel sent empty message!!\n");
		return NL_OK;
	}
	final_distance=nla_get_u32(attr[ATTR_MSG_4]);
	fprintf(stdout, "Kernel says distance is: %d \n", nla_get_u32(attr[ATTR_MSG_4]));

	return NL_OK;
}

// preparing socket
static void prep_nl_sock(struct nl_sock** nlsock)
{
	int family_id, grp_id=0;
	
	*nlsock = nl_socket_alloc();
	if(!*nlsock) {
		fprintf(stderr, "Unable to alloc nl socket!\n");
		exit(EXIT_FAILURE);
	}

	/* disable seq checks on multicast sockets */
	nl_socket_disable_seq_check(*nlsock);
	nl_socket_disable_auto_ack(*nlsock);

	/* connect to genl */
	if (genl_connect(*nlsock)) {
		fprintf(stderr, "Unable to connect to genl!\n");
		goto exit_err;
	}

	/* resolve the generic nl family id*/
	family_id = genl_ctrl_resolve(*nlsock, GENL_TEST_FAMILY_NAME);
	if(family_id < 0){
		fprintf(stderr, "Unable to resolve family name!\n");
		goto exit_err;
	}

	grp_id = genl_ctrl_resolve_grp(*nlsock, GENL_TEST_FAMILY_NAME,genl_test_mcgrp_names[0]);

	if (grp_id < 0)	{
		printf("Unable to resolve group name for!\n");
            goto exit_err;
		}
	if (nl_socket_add_membership(*nlsock, grp_id)) {
			printf("Unable to join group!\n"); 
            goto exit_err;
		}

    return;

exit_err:
    nl_socket_free(*nlsock); // this call closes the socket as well
    exit(EXIT_FAILURE);
}



//function to send message to kernel
static int send_msg_to_kernel(struct nl_sock *sock,void *f1,int message_num)
{
	struct nl_msg* msg;
	int family_id, err = 0;

	family_id = genl_ctrl_resolve(sock, GENL_TEST_FAMILY_NAME);
	if(family_id < 0){
		fprintf(stderr, "Unable to resolve family name!\n");
		exit(EXIT_FAILURE);
	}

	msg = nlmsg_alloc();
	if (!msg) {
		fprintf(stderr, "failed to allocate netlink message\n");
		exit(EXIT_FAILURE);
	}

	if(!genlmsg_put(msg, NL_AUTO_PID, NL_AUTO_SEQ, family_id, 0, 
		NLM_F_REQUEST, GENL_TEST_C_MSG, 0)) {
		fprintf(stderr, "failed to put nl hdr!\n");
		err = -ENOMEM;
		goto out;
	}
	if(message_num==ATTR_MSG_1)
		err = nla_put(msg, ATTR_MSG_1,sizeof(struct Commands_from_usr_1),f1);
	else if(message_num==ATTR_MSG_2)
		err = nla_put(msg, ATTR_MSG_2,sizeof(struct Commands_from_usr_2),f1);
	else if(message_num==ATTR_MSG_3)
		err = nla_put(msg, ATTR_MSG_3,sizeof(struct Commands_from_usr_3),f1);
	if (err) {
		fprintf(stderr, "failed to put nl message!\n");
		goto out;
	}

	err = nl_send_auto(sock, msg);
	if (err < 0) {
		fprintf(stderr, "failed to send nl message!\n");
	}

out:
	nlmsg_free(msg);
	return err;
}



//function executed by threads
void *myThreadFun(void *vargp) 
{ 
	pthread_mutex_lock(&lock); 
	final_distance=-1;

	//heart patterns
  	unsigned char user_pattern_1[1][8]={{0x1C,0x22,0x42,0x84,0x84,0x42,0x22,0x1C}};
	unsigned char user_pattern_2[1][8]={{0x1C,0x3E,0x66,0xC4,0xC4,0x66,0x3E,0x1C}};
	unsigned char user_pattern_3[1][8]={{0x1C,0x3E,0x7E,0xE4,0xE4,0x7E,0x3E,0x1C}};
	unsigned char user_pattern_4[1][8]={{0x1C,0x3E,0x7E,0xFC,0xFC,0x7E,0x3E,0x1C}};

	//diamonds pattern
	unsigned char user_pattern_1_1[1][8]={{0x18,0x24,0x42,0x81,0x81,0x42,0x24,0x18}};
	unsigned char user_pattern_2_2[1][8]={{0x18,0x3C,0x66,0xC3,0xC3,0x66,0x3C,0x18}};
	unsigned char user_pattern_3_3[1][8]={{0x18,0x3C,0x7E,0xE7,0xE7,0x7E,0x3C,0x18}};
	unsigned char user_pattern_4_4[1][8]={{0x18,0x3C,0x7E,0xFF,0xFF,0x7E,0x3C,0x18}};
	

	struct Commands_from_usr_1 *pat;
	struct Commands_from_usr_3 *do_mes;




	pat=malloc(sizeof(struct Commands_from_usr_1));
	do_mes=malloc(sizeof(struct Commands_from_usr_3));

	do_mes->request_measurement=1;



	cb = nl_cb_alloc(NL_CB_DEFAULT);
	nl_cb_set(cb, NL_CB_SEQ_CHECK, NL_CB_CUSTOM, skip_seq_check, NULL);
	nl_cb_set(cb, NL_CB_VALID, NL_CB_CUSTOM, print_rx_msg, NULL);


	prep_nl_sock(&nlsock);
	send_msg_to_kernel(nlsock,(void *)do_mes,3);
	nl_recvmsgs(nlsock, cb);
	prep_nl_sock(&nlsock);
	if(final_distance<15)  //if distance is less than 15cm print heart or else print diamond
		memcpy(pat->pattern.user_pattern,user_pattern_1,sizeof(user_pattern_1));
	else
		memcpy(pat->pattern.user_pattern,user_pattern_1_1,sizeof(user_pattern_1_1));
	send_msg_to_kernel(nlsock,(void *)pat,1);

	msleep(delay_fun());



	prep_nl_sock(&nlsock);
	send_msg_to_kernel(nlsock,(void *)do_mes,3);
	nl_recvmsgs(nlsock, cb);
	prep_nl_sock(&nlsock);
	if(final_distance<15) //if distance is less than 15cm print heart or else print diamond
		memcpy(pat->pattern.user_pattern,user_pattern_2,sizeof(user_pattern_1));
	else
		memcpy(pat->pattern.user_pattern,user_pattern_2_2,sizeof(user_pattern_1_1));
	send_msg_to_kernel(nlsock,(void *)pat,1);

	msleep(delay_fun());



	prep_nl_sock(&nlsock);
	send_msg_to_kernel(nlsock,(void *)do_mes,3);
	nl_recvmsgs(nlsock, cb);
	prep_nl_sock(&nlsock);
	if(final_distance<15) //if distance is less than 15cm print heart or else print diamond
		memcpy(pat->pattern.user_pattern,user_pattern_3,sizeof(user_pattern_1));
	else
		memcpy(pat->pattern.user_pattern,user_pattern_3_3,sizeof(user_pattern_1_1));
	send_msg_to_kernel(nlsock,(void *)pat,1);

	msleep(delay_fun());


	prep_nl_sock(&nlsock);
	send_msg_to_kernel(nlsock,(void *)do_mes,3);
	nl_recvmsgs(nlsock, cb);
	prep_nl_sock(&nlsock);
	if(final_distance<15) //if distance is less than 15cm print heart or else print diamond
		memcpy(pat->pattern.user_pattern,user_pattern_4,sizeof(user_pattern_1));
	else
		memcpy(pat->pattern.user_pattern,user_pattern_4_4,sizeof(user_pattern_1_1));
	send_msg_to_kernel(nlsock,(void *)pat,1);

	msleep(delay_fun());

	

	nl_cb_put(cb);
    nl_socket_free(nlsock);

    pthread_mutex_unlock(&lock); 

    return NULL; 
}






int main(int argc, char** argv)
{

	struct Commands_from_usr_2 *all_pins;
	all_pins=malloc(sizeof(struct Commands_from_usr_2));

	strcpy(all_pins->hcsr_pins.cmd_name,"haii hari hello");
	all_pins->hcsr_pins.trig=HCSR04_TRIGGER_PIN;
	all_pins->hcsr_pins.echo=HCSR04_ECHO_PIN;
	all_pins->spi_pins.spi_trig=MAX7219_CS_PIN;


	prep_nl_sock(&nlsock);
	send_msg_to_kernel(nlsock,(void *)all_pins,2);

	pthread_t thread_id_1,thread_id_2;


	if (pthread_mutex_init(&lock, NULL) != 0) { 
        printf("\n mutex init has failed\n"); 
        return 1; 
    } 
  
while(1){
	pthread_create(&thread_id_1, NULL, myThreadFun, NULL); 
	pthread_create(&thread_id_2, NULL, myThreadFun, NULL); 


}
	pthread_join(thread_id_1, NULL); 
	pthread_join(thread_id_1, NULL); 
	exit(0); 




}


//function to get delay from distance calculated 

long int delay_fun(void){

long int delay;
while(1){
	if(final_distance!=-1)
		break;
	//printf("final distance is %d\n",final_distance);
	msleep(1);
}
delay=final_distance*5;
final_distance=-1;
printf("delay in millisec %ld\n",delay);

return delay;
}



//function to get milliseconds sleep
int msleep(long msec)
{
    struct timespec ts;
    int res;

    if (msec < 0)
    {
        errno = EINVAL;
        return -1;
    }

    ts.tv_sec = msec / 1000;
    ts.tv_nsec = (msec % 1000) * 1000000;

    do {
        res = nanosleep(&ts, &ts);
    } while (res && errno == EINTR);

    return res;
}