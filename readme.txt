NAME DANDURI DATTA MANIKANTA SRIHARI
ASU ID 1217423272


To run the programm perform make operation and transfer files in to board using scp command.

server.ko is the kernel module
client is the application

My module printS 2 patterns 
1.Heart
2.Diamond

if distance measured is greater than 15cm diamond is printed or else heart is printed. Delay by which the animation changes depends as a function of distance. 

My module can be configured for multiple groups as well. In client application i implemented for 1 group.


SAMPLE OUTPUT TESTED FOR 
##################################################################################################################

TESTED FOR FOLLOWING PIN CONFIGURATION

#define MAX7219_CS_PIN 6
#define HCSR04_TRIGGER_PIN 7
#define HCSR04_ECHO_PIN 2

##################################################################################################################
root@quark:~# insmod server.ko
[ 5250.248789] genl_test: initializing netlink
[ 5250.265151] gpio-72 (MOSI_MUX2): _gpiod_direction_output_raw: missing set() or direction_output() operations
root@quark:~# ./client
[ 5252.178302] gpio-68 (pin_mux_68): _gpiod_direction_output_raw: missing set() or direction_output() operations
Kernel says distance is: 34
delay in microsec 170
Kernel says distance is: 34
delay in microsec 170
Kernel says distance is: 34
delay in microsec 170
Kernel says distance is: 34
delay in microsec 170
Kernel says distance is: 34
delay in microsec 170
Kernel says distance is: 34
delay in microsec 170
Kernel says distance is: 34
delay in microsec 170
Kernel says distance is: 34
delay in microsec 170
Kernel says distance is: 34
delay in microsec 170
Kernel says distance is: 34
delay in microsec 170
Kernel says distance is: 13
delay in microsec 65
Kernel says distance is: 13
delay in microsec 65
Kernel says distance is: 13
delay in microsec 65
Kernel says distance is: 13
delay in microsec 65
Kernel says distance is: 13
delay in microsec 65
Kernel says distance is: 13
delay in microsec 65
Kernel says distance is: 13
delay in microsec 65
Kernel says distance is: 13
delay in microsec 65
Kernel says distance is: 34
delay in microsec 170
^C
root@quark:~# ^C
root@quark:~# ^C
root@quark:~# rmmod server
################################################################################################################################################
