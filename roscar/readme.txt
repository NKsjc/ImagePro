编程日志：
1.串口数据传输中断问题解决：
现象：传输的数据不定时地中断，但当接收的数据固定不变时不会出问题。
分析：可能是遇到特殊的字符，串口数据暂停。
原因: 没有屏蔽软件流控。
解决方法：c_iflag &= ~(IXON | IXOFF | IXANY);//关闭软件流控

2.通信周期较短时数据连包。
现象：
分析：
问题：接收没有逐个字节按照协议格式进行处理。
