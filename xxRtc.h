#ifndef XXRTCH
#define XXRTCH

#define rtcRead		0x00
#define rtcWrite	0x80

#define arSecond	0x00
#define arMinute	0x01
#define arHour		0x02
#define arDayOfWeek	0x03
#define arDay		0x04
#define arMonth		0x05
#define arYear		0x06

#define arControl	0x0E
#define arStatus	0x0F
#define arAgeOffset	0x10
#define arTemperature	0x11

#define arRamAddress	0x18
#define arRamData	0x19

#define ctOscDisable	0x80
#define ctBatWaveEnable	0x40
#define ctTempConvert	0x20
#define ctRate1Hz	0x40
#define ctRate1024Hz	0x08
#define ctRate4096Hz	0x10
#define ctRate8192Hz	0x18
#define ctIntAlarmEnable	0x04
#define ctAlarm2Enable	0x02
#define ctAlarm1Enable	0x01

#define stOscStopFlag	0x80
#define stBat32KHzEna	0x40
#define stConvRate64	0x00
#define stConvRate128	0x10
#define stConvRate256	0x20
#define stConvRate512	0x30
#define st32KHzEna	0x08
#define stBusy  	0x04
#define stAlarm2Flag	0x02
#define stAlarm1Flag	0x01


#endif

