ad-temprature:使用内部的一个AD温度传感器,采集温度并通过OLED显示出来,驱动为系统提供的rit128x96x4,用到了AD中断和计数器中断.
blinky:控制一个LED灯闪动
button:通过按键中断来改变一个LED灯的状态,使用到了GPIOF中断.
graphics:在OLED上显示图形,使用了系统提供的rit128x96x4驱动.
hello:使用LM3S8962板子上的OLED,通过系统提供的rit128x96x4驱动对OLED进行初始化,并显示hello world.
irreader:读取空调红外信号并打印出来.
irsender:简单的空调红外发射demo
lm3s-server:
scope:AD对电压采样并在OLED上显示出来,使用自己写的oled驱动,用到了AD中断和计数器中断.
testbss:只编译,不运行,看看data段和bss段是如何生成的.
watchdog:测试看门狗中断,在看门狗中断中翻转LED状态.
