#include <linux/kernel.h>
#include <linux/syscalls.h>

extern void connect_port_priority(int priority, int port);

SYSCALL_DEFINE2(my_syscall, int, priority, int, port) {
	if(port % 2 == 1)
		connect_port_priority(priority, port);
	return 0;
}

