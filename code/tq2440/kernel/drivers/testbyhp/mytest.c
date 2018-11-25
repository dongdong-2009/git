#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>

int mytest_func(void)
{
	printk("fuck mytest_func!\n");
}

void mytest_func_exit(void)
{
	printk("fuck exit!\n");
}


module_init(mytest_func);
module_exit(mytest_func_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("HP");
MODULE_VERSION("V1.0");
MODULE_DESCRIPTION("This is a queue module !");
