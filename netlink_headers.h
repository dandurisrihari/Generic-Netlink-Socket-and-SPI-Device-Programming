
#ifndef GENL_TEST_H
#define GENL_TEST_H


#ifndef __KERNEL__
#include <netlink/genl/genl.h>
#include <netlink/genl/family.h>
#include <netlink/genl/ctrl.h>
#endif

#define GENL_TEST_FAMILY_NAME		"my_netlink"
#define GENL_TEST_MCGRP0_NAME		"client_1"



struct config_pins {
  char cmd_name[20];
  int trig;
  int echo;
};


struct spi_pins_set {
  int spi_trig;
};

struct user_pattern_data {
    unsigned char user_pattern[1][8];
};


//pattern datastructure
struct Commands_from_usr_1{
	struct user_pattern_data pattern;

};

//pins datastructure
struct Commands_from_usr_2{
	struct config_pins hcsr_pins;
	struct spi_pins_set spi_pins;	

};
//perform distance calculation datastructure
struct Commands_from_usr_3{
	int request_measurement;

};

enum {
	GENL_TEST_C_UNSPEC,		/* Must NOT use element 0 */
	GENL_TEST_C_MSG,
	//sGENL_TEST_C_MSG_2,
};

enum genl_test_multicast_groups {
	MCGRP0,
	// MCGRP1,
	// MCGRP2,
};
#define GENL_TEST_MCGRP_MAX		1



enum genl_test_attrs {
	GENL_TEST_ATTR_UNSPEC,		/* Must NOT use element 0 */

	ATTR_MSG_1,
	ATTR_MSG_2,
	ATTR_MSG_3,
	ATTR_MSG_4,

	__GENL_TEST_ATTR__MAX,
};
#define GENL_TEST_ATTR_MAX (__GENL_TEST_ATTR__MAX - 1)

static struct nla_policy genl_test_policy[GENL_TEST_ATTR_MAX+1] = {
	[ATTR_MSG_1] = {
		.type = NLA_NESTED,
#ifdef __KERNEL__
		//.len = sizeof(struct Commands_from_usr_1)
#else
		//.maxlen = sizeof(struct Commands_from_usr_1)
#endif
	},

	[ATTR_MSG_2] = {
		.type = NLA_NESTED,
#ifdef __KERNEL__
		//.len = sizeof(struct Commands_from_usr_1)
#else
		//.maxlen = sizeof(struct Commands_from_usr_1)
#endif
	},

		[ATTR_MSG_3] = {
		.type = NLA_NESTED,
#ifdef __KERNEL__
		//.len = sizeof(struct Commands_from_usr_1)
#else
		//.maxlen = sizeof(struct Commands_from_usr_1)
#endif
	},

	[ATTR_MSG_4] = {
		.type = NLA_U32,
#ifdef __KERNEL__
		.len = sizeof(unsigned int)
#else
		.maxlen = sizeof(unsigned int)
#endif
	},

};

#endif
