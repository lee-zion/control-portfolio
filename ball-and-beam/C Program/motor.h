/* Defined Enumerators */

enum LogicFlag		 { NOK, YOK };
enum Motor_Dir_FLAG  { STOP, CW, CCW, IDLE };

/* Unit */

#define CMD						(double)(	10.0					) // [cm]
#define PI						(double)(	3.14159265358979		)
#define N_AI					(int)(		3						)
#define g						(double)(	9.80665					)

/* Fading Memory Filter */

#define N_STATE					(int)	(	3						) // [-]
#define BETA					(double)(	0.91					) // [-]
#define CONVERGENCE_TIME		(double)(	2.0						) // [s]

/* Simulation Time */

#define SAMPLING_FREQ			(double)(	200.0					) // [Hz]
#define SAMPLING_TIME			(double)(	1.0/SAMPLING_FREQ		) // [s]
#define LASTTIME				(double)(	15						) // [s]

#define N_STEP					(int)	(	SAMPLING_FREQ*LASTTIME	) // [-]

#define START_AVG_TIME			(double)(	CONVERGENCE_TIME + 1.0			)
#define START_AVG_STEP			(int)	(	SAMPLING_FREQ*START_AVG_TIME	)

/* Motor Linearization */

#define SAT_POINT_CW			(double)(	2.1					) // Vc minus
#define DEAD_POINT_CW			(double)(	1.3					) // 33
#define SAT_POINT_CCW			(double)(	2.0					)
#define DEAD_POINT_CCW			(double)(	1.3					) // 31
#define V_MAX					(double)(	5.0					) // Max Vc
#define V_MIN					(double)(	-5.0				)

/* Observer Beam */

#define O_WM					(double)(	23.9389					)
#define O_L1					(double)(	-0.77					)
#define O_L2					(double)(	-0.5723					)
#define O_L3					(double)(	8.3315					)
#define O_L4					(double)(	191.0706				)

#define C1						(double)(	5.0*g/9.0				)

/* Beam Unit */

#define V2D_P1					(double)(	-34.07					)
#define V2D_P2					(double)(	88.54					)

#define D2LV_P1_CCW				(double)(	0.04681					)
#define D2LV_P2_CCW				(double)(	0.0						)

#define D2LV_P1_CW				(double)(	0.04671					)
#define D2LV_P2_CW				(double)(	0.0						)

/* Ball	Unit */

#define V2X_P1					(double)(	0.0605					)
#define V2X_P2					(double)(	-0.1514					)

#define m						(double)(	0.11					) // [kg]
#define Jbeam					(double)(	0.0022					) // [kg m2]
#define	Km						(double)(	1.2204					)
#define A1						(double)(	-O_WM					)
#define A2						(double)(   m*g/Jbeam				)
#define B						(double)(	O_WM*Km					)

/* Ackermann's Full State Feedback Control */

#define K1						(double)(	26.45					)	
#define K2						(double)(	0.8325					)	
#define K3						(double)(	131.0					)	
#define K4						(double)(	31.78					)	
#define N_BAR					(double)(   B/(B*K3 - A2)			)

/* LQR Full State Feedback Control */

// #define K1						(double)(	26.5143					)	
// #define K2						(double)(	1.2291					)	
// #define K3						(double)(	118.3429				)	
// #define K4						(double)(	31.4202					)	
// #define N_BAR					(double)(   B/(B*K3 - A2)			)

/* LQServo Full State Feedback Control */

#define Ka11					(double)(	26.6956					)	
#define Ka12					(double)(	1.6903					)	
#define Ka13					(double)(	88.4430					)	
#define Ka14					(double)(	25.0355					)
#define	Ka2						(double)(	-81.6497				)

/* Macro Functions */

#define RAD2DEG(x)				(double)(	(x)*180.0/PI				)
#define DEG2RAD(x)				(double)(	(x)*PI/180.0				)

#define	VOL2DEG(x)				(double)(	V2D_P1 * (x) + V2D_P2		)
#define	VOL2RAD(x)				(double)(	VOL2DEG(x) * PI / 180.0		)

#define	DEG2LVOL_CCW(x)			(double)(	D2LV_P1_CCW * (x) + D2LV_P2_CCW		)
#define	DEG2LVOL_CW(x)			(double)(	D2LV_P1_CW * (x) + D2LV_P2_CW		)

#define LINEAR_CW(x)			(double)(	(SAT_POINT_CW - DEAD_POINT_CW)    / 5.0 * (x) + DEAD_POINT_CW	)
#define LINEAR_CCW(x)			(double)(	(SAT_POINT_CCW - DEAD_POINT_CCW)  / 5.0 * (x) + DEAD_POINT_CCW	)

#define VOL2X(x)				(double)(	V2X_P1 * (x) + V2X_P2		)

/*	Local Global Variables */

FILE*				pFile;

TaskHandle			AI_S;		// 0: x, 1: gyro, 2: pot
TaskHandle			AO_Vcmd;
TaskHandle			DO_Dir;

static int			Stop_FLAG = 0;
static int			Stop_Count = 0;

static int			Tcount;
static int			flagInit;

static double		Time;
static double		StartTime;
static double		EndTime;

static double		chk_real;

static int			Motor_Dir = 2;	// 01: CW, 10: CCW
static int			Pre_Dir	  = 0.0;

static double		x_In = 0.0;
static double		g_In = 0.0;
static double		p_In = 0.0;

static double		Vc	 = 0.0;		// -5   ~ 5
static double		Vc_L = 0.0;

static double		mAng		= 0.0;
static double		mX			= 0.0;
static double		prev_mX		= 0.0;
static double		x_cmd		= 0.0;
static double		prev_x_cmd	= 0.0;
static double		u			= 0.0;
static double		prev_u		= 0.0;
static double		Vavg		= 0.0; 

static double		Wcmd		= 0.0;

static double		Residue_beam = 0.0;
static double		Residue_ball = 0.0;

static double		fA = 0.0;
static double		fW = 0.0;
static double		fX = 0.0;
static double		fV = 0.0;

static double		eA = 0.0;
static double		eW = 0.0;
static double		eX = 0.0;
static double		eV = 0.0;

static double		X_hat_curr[3] = { 0.0 };
static double		X_hat_prev[3] = { 0.0 };

static double		buffA[N_STEP] = { 0.0 };
static double		buffW[N_STEP] = { 0.0 };
static double		buffX[N_STEP] = { 0.0 };
static double		buffV[N_STEP] = { 0.0 };

static double		bufeA[N_STEP] = { 0.0 };
static double		bufeW[N_STEP] = { 0.0 };
static double		bufeX[N_STEP] = { 0.0 };
static double		bufeV[N_STEP] = { 0.0 };

static double		bufTime[N_STEP] = { 0.0 };
static double		bufcmd[N_STEP]	= { 0.0 };
static double		bufmV[N_STEP]	= { 0.0 };
static double		bufmAng[N_STEP] = { 0.0 };
static double		bufmX[N_STEP]	= { 0.0 };

static double		getvoltage[N_AI]	= { 0.0, };   // 0: x, 1: w, 2: pot
static double		Kf[N_STATE]			= { 0.0, };

static double		X_hat_beam[N_STATE] = { 0.0, };
static double		X_bar_beam[N_STATE] = { 0.0, };

static double		X_hat_ball[N_STATE] = { 0.0, };
static double		X_bar_ball[N_STATE] = { 0.0, };

static char			Text_Name[100] = { NULL, };

/*	Functions */

void Initialize(void);
void GetFilterGain(void);

void Check_RealTime(void);
void CheckTime(void);
double GetWindowTime(void);

// FMF Function
void GetMeasurement(void);
void MeasUpdate(void);
void TimeUpdate(void);

// Observer Function
void Observer(void);

// Nbar, LQR or LQ Servo
void Controller(void);
void Nbar(void);
void LQServo(void);

double Command(void);

void SaveData(void);
void IdleTime(void);

void Close(void);