.global _start		@声明，在链接器脚本中使用
.global light_led	@声明，在C文件中调用

@定义7个异常向量表
_start:
	b RESET
	@ldr pc, =RESET
	ldr pc, _Undefined_instruction
	ldr pc, _Software_interrupt
	ldr pc, _Prefetch_abort
	ldr pc, _Data_abort
	ldr pc, _Unused                 @即使未使用也要要保留占位
	ldr pc, _IRQ					@否则下面的IRQ和FIQ会出错
	ldr pc, _FIQ

@标号与函数地址一一对应
_Undefined_instruction : .word Undefined_instruction
_Software_interrupt    : .word Software_interrupt
_Prefetch_abort        : .word Prefetch_abort
_Data_abort            : .word Data_abort
_Unused                : .word Unused
_IRQ                   : .word IRQ
_FIQ                   : .word FIQ

@各个函数的具体操作
Undefined_instruction : nop
Software_interrupt    : nop
Prefetch_abort        : nop
Data_abort            : nop
Unused                : nop
FIQ                   : nop

@只要是发生了中断，无论是什么中断，都会跳转到此处执行代码
IRQ:
	sub lr, lr, #4    			@首先计算出下一条指令的位置
	stmfd sp!, {r0-r12, lr}		@将r0到r12寄存器和lr寄存器压入堆栈
	bl handle_init           	@跳转到函数，该函数实现判断中断源并调用中断服务函数的作用
	ldmfd sp!, {r0-r12, pc}^ 	@弹栈，注意lr现在要弹出到pc ^表示把spsr恢复到cspr里面

RESET:
	bl set_svc                @在此处就进入了svc模式，以下的操作都是在svc模式下
	bl disable_watchdog
	bl mask_all_int
	bl disable_mmu_cache
	bl clock_init
	bl sdram_init
	bl stack_init             @因为下面的要调用c函数，所以要把初始化栈向前移动
	bl nand_init              @调用c语言里面的代码，nand初始化
	bl copy_mem               @这里有错但是还找不到在哪，找到了，内存被覆盖了

	bl bss_init
	@bl light_led			  @调试代码使用
	b goto_main

@设置处理器为svc模式
set_svc:
	mrs r0, cpsr
	bic r0, r0, #0x1f
	orr r0, r0, #0xd3        @在设置svc模式的同时
	msr cpsr, r0			 @关闭了IRQ和FRQ中断
	mov pc, lr				 @返回

@关闭看门狗
#define pWTCON 0x53000000
disable_watchdog:
	ldr r0, =pWTCON			@传入寄存器的地址
	mov r1, #0x00
	str r1, [r0]			@直接将该寄存器写入0
	mov pc, lr

@屏蔽所有中断
#define pMASK 0x4A000008
mask_all_int:
	ldr r0, =pMASK
	mvn r1, #0x0              @取反后为全F
	str r1, [r0]              @直接写寄存器
	mov pc, lr                @返回

@关闭mmu的ID cache
disable_mmu_cache:
	mcr p15, 0, r0, c7, c7, 0 @flush ID cache，参见ARM核手册P42
	mrc p15, 0, r0, c1, c0, 0 @先读出control寄存器 disable
	ldr r1, =0x07             @bic立即数不能大于8位
	bic r0, r0, r1            @将第0、2、12位清零，关闭mmu和ID cache
	mcr p15, 0, r0, c1, c0, 0 @再写回
	mov pc, lr                @返回

@设置时钟分频
#define CLKDIVN 0x4C000014
#define MPLLCON 0x4C000004
#define MPLL_405MHZ ((127<<12)|(2<<4)|(1<<0))
clock_init:
	ldr r0, =CLKDIVN          @设置分频系数
	mov r1, #0x05             @参考u-boot，配置FCLK:HCLK:PCLK = 1:4:8
	str r1, [r0]              @写入寄存器

	mrc p15, 0, r0, c1, c0, 0 @设置异步工作模式
	orr r0, r0, #0xC0000000   @参见芯片手册P243
	mcr p15, 0, r0, c1, c0, 0

	ldr r0, =MPLLCON          @设置FCLK
	ldr r1, =MPLL_405MHZ      @设置为405MHZ FCLK由MPLL管理
	str r1, [r0]              @写入寄存器

	mov pc, lr

@SDRAM初始化
@要写入寄存器的值
mem_data:
	.long 0x22000000
	.long 0x00000700
	.long 0x00000700
	.long 0x00000700
	.long 0x00000700
	.long 0x00000700
	.long 0x00000700
	.long 0x00018001
	.long 0x00018001
	.long 0x008c04f5
	.long 0x000000b1
	.long 0x00000030
	.long 0x00000030

@定义第一个寄存器的地址
#define mem_contrl 0x48000000
sdram_init:
	ldr r0, =mem_contrl
	add r3, r0, #4*13      @r3表示写完13个寄存器后，结束地址
	adrl r1, mem_data      @加载mem_data的地址到r1

0:                         @循环体
	ldr r2, [r1], #4
	str r2, [r0], #4
	cmp r0, r3
	bne 0b                 @b表示向前跳转

	mov pc, lr

@代码搬移  下面的是从IRAM中直接搬移的
@copy_mem:
@	ldr r0, =0x00000000    @从0地址开始拷贝
@	ldr r1, =0x30008000    @要搬移到的地址处
@	add r3, r0, #1024*4    @要搬移4K的代码，r3里面的是结束地址
@
@copy_loop:
@	ldr r2, [r0], #4       @从地址r0处读出数据到r2，然后r0加4
@	str r2, [r1], #4       @然后将刚读取到的值存放到r1对应的地址中
@	cmp r0, r3             @比较地址，看是否取完了
@	bne copy_loop
@
@	mov pc, lr

@代码搬移直接从NAND中搬移程序到内存
copy_mem:
	mov r0, #0             @从nand的地址0处开始拷贝数据 即参数start_addr
	ldr r1, =_start        @要搬移到内存里面的数据，即参数sdram_addr
	ldr r2, =bss_end       @程序的结束地址
	sub r2, r2, r1         @计算整个程序的大小，得要搬移的到字节数，即size参数

	mov ip, lr             @在调用函数之前需要保存lr的值，ip既为r12寄存器
	bl nand_to_ram         @调用C语言编写的函数，参数通过r0，r1，r2寄存器传递
	mov lr, ip             @恢复lr
	mov pc, lr             @返回




@栈初始化1 在svc模式下
@stack_init:
@	ldr sp, =0x34000000		@这里初始化的sp指针是在svc模式下的sp指针
@	mov pc, lr				@实际上是初始化r13_svc寄存器

@栈初始化2 在IRQ模式下
stack_init:
	msr cpsr_c, #0xd2        @进入到中断IRQ模式下
	ldr sp, =0x33000000

	msr cpsr_c, #0xd3        @再次进入到svc模式
	ldr sp, =0x34000000
	mov pc, lr

@bss段初始化
bss_init:
	ldr r0, =bss_start    @这两个变量在连接器脚本里面定义
	ldr r1, =bss_end
	cmp r0, r1
	moveq pc, lr          @若bss空间大小为0，则返回
bss_loop:
	mov r2, #0
	str r2, [r0], #4      @向bss段写入0，然后地址加4
	cmp r0, r1
	blo bss_loop

	mov pc, lr

@点亮LED 对应GPB5、6、7、8
#define GPBCON 0x56000010
#define GPBDAT 0x56000014
light_led:
	ldr r0, =GPBCON           @先把GPBCON的地址存到r0里面去
	ldr r1, =0x15400          @把要配置的值存放到r1里面
	str r1, [r0]              @写入寄存器，设置为输出

	ldr r0, =GPBDAT           @再把GPBDAT寄存器的地址存到r0
	ldr r1, =0x35f            @只把led0了led2点亮
	str r1, [r0]              @点亮LED

	mov pc, lr                @返回

@跳转到C环境下
goto_main:
	ldr pc, =my_main          @直接修改pc指针，下面就没有返回了

