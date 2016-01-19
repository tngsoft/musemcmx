#ifndef CustomH
#define CustomH

#define TOOLNAME	"›Mœ¡"
#define RUSSNAME	"›Mœ¡"

#define ANYADDRESS	0xFF
#define TOOLADDRESS	0x07
#define TOOLTIME	0x81
#define TOOLREPLY       0x03

#define NUMBLOCKS       2097152
#define SIZEPAGE        512
#define NUMPAGES        1
#define SIZEPAGEW       256
#define SIZEPAGE        512
#define VERSION	        0x0310
#define F_Crystal 	7372800

#define SIZEOFDATA      406  // 8+2 + 4*19 + 3*4
#define TOOLIDENT       0x0A22

#define ONERECORD       3600


typedef struct {
    char Ident[4];
    int BaudRate485;
    int Div485;
    float A[3];
    float B[3];
    float C[3];
    float AirNull[3];
    float ZondNull[3];
    float TA[3], TB[3], TC[3];
    short int Reserv[8];
} TCalibrate;

extern TCalibrate Calb;

typedef struct {
    TRtcTime rt;//8
    long int  mark;//4
    float Volt;//4
    float Temp;//4
    short int Z;//2
    float dPhase[3];//12
    float Rp[3];//12
    //46
    short int Sin1[90]; //120
    short int Sin2[90]; //120
    //406
} TRecord;

extern float B123[8];
extern char Reply485[8];

// CS for ADC
#define PortCtrl	GPIOB
#define CSAdc		1

// Enable Currents I1 & I2
#define Gen1Port	GPIOC
#define Gen2Port	GPIOC
#define Gen3Port	GPIOE
#define PinG1		7
#define PinG2		6
#define PinG3		5

#define sinPeriodPoints 40
#define sinPeriodCount 10
#define sinTotalPoints sinPeriodPoints*sinPeriodCount

extern float Sin1f[sinTotalPoints],
             Sin2f[sinTotalPoints];

void PowerOn( void );
void PowerOff( void );
void BreakFunction( void );

void InitCalb( void );
void InitCustom( void );
void ConvertData( void );
void CustomWork( int param );
void CustomRtcIntr( void );

int ReadAdc( short int *s1, short int *s2, int words );
void Average(short int *src, float *dest);
void filter(float *buf);
float GetDeltaPhase( float *sin1, float *sin2 );

#endif