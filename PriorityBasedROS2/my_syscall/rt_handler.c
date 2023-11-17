#include <linux/kernel.h>
#include <linux/syscalls.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/percpu.h>
#include <linux/cpu.h>
#include <linux/kthread.h>
#include <linux/sched.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/timekeeping.h>
#include <linux/seq_file.h>
#include <linux/delay.h>


MODULE_LICENSE("Dual BSD/GPL");

struct task_struct *krtd;
struct task_struct *kmond;
extern int get_highest_prio(void);
extern void net_rt_handler_action(void);

DEFINE_SPINLOCK(priority_lock);
DEFINE_SPINLOCK(set_lock);


static int run_monitor(void* arg)
{
	unsigned long flags;
	int msg_priority, krtd_priority;
	struct sched_param param;

	for (;;) {
		spin_lock_irqsave(&set_lock, flags);

		msg_priority = get_highest_prio();
		krtd_priority = krtd->rt_priority;
	
		if(msg_priority > krtd_priority) {
			param.sched_priority = msg_priority;
			sched_setscheduler(krtd, SCHED_FIFO, &param);
		}

		if (krtd->state != TASK_RUNNING){
			wake_up_process(krtd);
		}
	
		spin_unlock_irqrestore(&set_lock, flags);

		set_current_state(TASK_INTERRUPTIBLE);
		schedule();
		set_current_state(TASK_RUNNING);	
		
		if(kthread_should_stop())
			break;
	}
	return 0;
}


static int run_rt_handler(void* arg)
{
	unsigned long flags;
	int cur_priority = 1;
	struct sched_param param;
	int check = 0;

	for (;;) {
		spin_lock_irqsave(&priority_lock, flags);

		//check = current->rt_priority;
		
		net_rt_handler_action();

		spin_unlock_irqrestore(&priority_lock, flags);

		cur_priority = get_highest_prio();
		param.sched_priority = cur_priority;
		sched_setscheduler(current, SCHED_FIFO, &param);		

		if (current->rt_priority < 50) {
			set_current_state(TASK_INTERRUPTIBLE);
			schedule();
			set_current_state(TASK_RUNNING);
		}
		if (kthread_should_stop())
			break;
		
	}

	return 0;
}

static int rt_handler_init(void)
{
	struct sched_param param;
	unsigned int cpu;
	cpumask_t mask;
	printk("rt_handler init start\n");

	krtd = kthread_run(run_rt_handler, NULL, "krtd");
	if (!krtd) 
		goto krtd_err;

	kmond = kthread_run(run_monitor, NULL, "kmond");
	if (!kmond)
		goto kmond_err;

	param.sched_priority = 99;
	if (sched_setscheduler(kmond, SCHED_FIFO, &param)) {
		printk(KERN_ERR "Failed to set kmond priority\n");
		goto err;
	}
	
	//pin cpu 0
	cpumask_clear(&mask);
	cpumask_set_cpu(0, &mask);
	if (set_cpus_allowed_ptr(krtd, &mask)) {
		printk(KERN_ERR "Failed to pinning krtd\n");
		goto err;
	}
	if (set_cpus_allowed_ptr(kmond, &mask)) {
		printk(KERN_ERR "Failed to pnning kmond\n");
		goto err;
	}

	printk("rt_handler init end\n");
	return 0;

err:
	kthread_stop(kmond);
kmond_err:
	printk(KERN_ERR "kmond created failed\n");
	kthread_stop(krtd);
krtd_err:
	printk(KERN_ERR "krtd created failed\n");
	return 0;
	
}

static void rt_handler_exit(void)
{
	kthread_stop(krtd);
	kthread_stop(kmond);
	printk("rt_handler exit\n");
}

module_init(rt_handler_init);
module_exit(rt_handler_exit);
