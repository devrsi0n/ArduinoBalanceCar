/****1****/
#Date: 2014.11.26
@author: qianmofeiyu
Description: This is our balance car code.
	Because of hardware problem, first we just want to make the car self balancing, 
	without bluetooth control and more complex code.

/****2****/
#Date: 2014.11.27
@author: qianmofeiyu
Description: fix many logic bugs, tested motors, read mup6050 ok, angle filter ok, 
	motors control output ok(include direction). But not test bluetooth,or EEPROM or encoders.
	(Just modify angle control PID argments by manual operation that download firmware every time).

/***3***/
#Date: 2014.12.2
@author: qianmofeiyu
Description: add more comments, add code that caculate car direction by encoders,
	and adjust speed sample period & speed control period.
	
/***4***/
#Date: 2014.12.7
@author: qianmofeiyu
Description: add angle PID args --> I, speed PID args --> D; add gyro filter, now final angle is smoother; 
	fix some spell bugs.


