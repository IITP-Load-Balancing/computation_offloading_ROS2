#include <linux/percpu.h>
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

struct task_struct *knonrtd;
extern void net_non_rt_handler_action(void);

//file
#define FILE_PATH "/home/final/test/experiment4_NonRT/Non_RTHandler.txt"
struct file *f;
static int write_to_file(char *buf, int len) {
    mm_segment_t oldfs;
    int ret;

    oldfs = get_fs();
    set_fs(get_ds());
    ret = vfs_write(f, buf, len, &f->f_pos);
    set_fs(oldfs);

    return ret;
}

static int run_non(void* arg)
{
	u64 start, end, elapsed;
	start = ktime_get_ns();
	/*
	char buf[256];
	int len;
	f = filp_open(FILE_PATH, O_WRONLY|O_CREAT, 0644);

	if (IS_ERR(f)) {
	    printk(KERN_ERR "Failed to open the file\n");
	    return PTR_ERR(f);
	}
	*/

	for (;;) {
	    net_non_rt_handler_action();

	    end = ktime_get_ns();
	    elapsed = end - start;

	    
	    if (elapsed >= msecs_to_jiffies(2) * (NSEC_PER_SEC / HZ)) {
		u64 elapsed_nsec = elapsed % 1000000000;
		u64 elapsed_sec = elapsed / 1000000000;
		printk("Elapsed time: %llu.%09llu s\n", elapsed_sec, elapsed_nsec);
		unsigned int remaining = 5 - 2;
		//printk("Sleep non-rt thread\n");
		msleep(remaining);
		//schedule_timeout_interruptible(msecs_to_jiffies(remaining));
		start = ktime_get_ns();
	    }
	    
	    //u64 elapsed_nsec = elapsed % 1000000000;
	    //u64 elapsed_sec = elapsed / 1000000000;
	    //u64 elapsed_nsec = elapsed % 1000000000;
	    //len = sprintf(buf, "Elapsed time: %llu.%09llu s\n", elapsed_sec, elapsed_nsec);
	    //write_to_file(buf, len);
	    

	    //schedule_timeout_interruptible(0.0125*HZ);
	    if (kthread_should_stop())
		break;
	}

	//filp_close(f, NULL);
	return 0;
}

static int non_init(void)
{
	int priority = 99;
	struct sched_param param;
	cpumask_t mask;
	param.sched_priority = priority;

	printk("module init start\n");
	
	//create kernel thread
	knonrtd = kthread_run(run_non, NULL ,"knonrtd");
	if (!knonrtd) {
		printk(KERN_ERR "knonrtd create failed\n");
		return 0;
	}
	
	//pin cpu 0
	cpumask_clear(&mask);
	cpumask_set_cpu(0, &mask);
	if (set_cpus_allowed_ptr(knonrtd, &mask)){
		printk(KERN_ERR "knonrtd fail to pinning\n");
		return 0;
	}

	//assign priority
	if (sched_setscheduler(knonrtd, SCHED_FIFO, &param)){
		printk(KERN_ERR "knonrtd fail to assign priority\n");
		return 0;
	}

	
	printk("module init end\n");
	return 0;
}

static void non_exit(void)
{
	kthread_stop(knonrtd);
	printk("non_rt_handler exit\n");
}

module_init(non_init);
module_exit(non_exit);

