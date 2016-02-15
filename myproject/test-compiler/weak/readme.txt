如何使用weak attribute的属性

在用gcc进行cortex-m3进行开发的时候,发现使用中断服务函数很麻烦.gcc工程里的中断向量表在文件startup_gcc.c里给出.但是在这个向量表里,只给出了Reset_Handler,NmiSR,FaultISR等几个必须的中断向量标志,其他的中断向量都被指向了IntDefaultHandler,这样的情况下,如果要使用这些器件中断,就必须修改startup_gcc.c文件.而且是每次使用不同的中断都要修改,特别麻烦.
	那么有没有更好的方法来解决这一问题呢.答案是有的.首先我们来看一个小程序:

//weak.c
#include<stdio.h>

extern void foo(void) __attribute__((weak));

void foo(void)
{
	printf("weak.\n");
}

int main(void)
{
	foo();
}


//strong.c
#include<stdio.h>
void foo(void)
{
	printf("strong.\n");
}

大家可以看到,weak.c里有一个成员函数foo(),但是属性是weak,同时有一个主函数.
而strong.c里只有一个foo()函数.

下面我们先编译weak.c:
	gcc -c -o weak.o weak.c
然后链接:
	gcc -o weak weak.o
最后运行weak:
	./weak
输出显而易见:weak.

现在我们编译strong.c:
	gcc -c -o strong.o strong.c
然后链接strong:
	gcc -o strong weak.o strong.o
这时候我们再来运行strong:
	./strong
输出结果是:strong.
不知道大家会不会觉得奇怪,两个源文件里都有一个foo()函数,这样难道不会冲突吗?
最后一次链接的时候,两个目标文件里各有一个foo()函数,我们可以通过nm命令来查看:
nm weak.o strong.o
输出:
weak.o:
00000000 W foo
00000014 T main
         U puts

strong.o:
00000000 T foo
         U puts
可以看到weak.o里的foo函数的属性是W,就是weak的意思,当链接器在链接的时候,发现了优先级更高的strong.o里的foo函数,就不再使用weak.o里的弱属性foo()函数了.


但是这和我们的cortex-m3开发有什么关系?
当然有了,我们可一在startup_gcc.c里添加具有weak属性的中断函数服务程序,当工程里不使用这些中断时,链接器就链接startup_gcc.c里默认的中断服务程序(死循环).
当工程里使用了这些中断,只需要在工程函数里添加中断服务函数,而不需要再对startup_gcc.c这个启动文件进行修改了.

比如,我要用ADC0转换完毕中断,那么只需要提前在startup_gcc.c里的适当位置添加
extern void ADC0IntHandler(void) __attribute__((weak));
和
void ADC0IntHandler(void)
{
	while(1)
	{}
}

同时在我的工程C文件里添加:
void ADC0IntHandler(void)
{
	真正的中断服务程序;
}

如果不需要使用ADC0中断时,仅仅删除工程C文件里的这个函数就行了.

