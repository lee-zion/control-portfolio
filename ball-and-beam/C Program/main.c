#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <windows.h>
#include "NIDAQmx.h"
#include "Motor.h"

void main(void)
{
	Initialize();

	do
	{
		x_cmd = CMD / 100.0;

		Check_RealTime();
		CheckTime();

		GetMeasurement();

		// FMF
		MeasUpdate();
		TimeUpdate();

		Observer();

		Controller();

		if (Tcount > CONVERGENCE_TIME*SAMPLING_FREQ)
			Command();

		SaveData();
		IdleTime();
	} while (Time < LASTTIME);

	Close();
}

void Initialize(void)
{
	if (!flagInit)
	{
		GetFilterGain();

		DAQmxCreateTask("", &AI_S);
		DAQmxCreateTask("", &AO_Vcmd);
		DAQmxCreateTask("", &DO_Dir);

		DAQmxCreateAIVoltageChan(AI_S, "Dev1/ai0:2", "", DAQmx_Val_RSE, -10.0, 10.0, DAQmx_Val_Volts, ""); // 0: x, 1: w, 2: pot
		DAQmxCreateAOVoltageChan(AO_Vcmd, "Dev1/ao0", "", 0.0, 5.0, DAQmx_Val_Volts, ""); // -Vc: CW, Vc: CCW
		DAQmxCreateDOChan(DO_Dir, "Dev1/port0:1", "", DAQmx_Val_ChanForAllLines); // 01: CW, 10: CCW

		DAQmxStartTask(AI_S);
		DAQmxStartTask(AO_Vcmd);
		DAQmxStartTask(DO_Dir);

		DAQmxWriteAnalogScalarF64(AO_Vcmd, 1, 5, 0.0, NULL);
		DAQmxWriteDigitalScalarU32(DO_Dir, 1, 1 / SAMPLING_FREQ, Motor_Dir, NULL);

		flagInit  = YOK;
		chk_real  = YOK; 
		Motor_Dir = CCW; 
		Tcount	  = 0; 

		printf("Press Enter to start.....");
		getchar();
	}
	else
	{
		return;
	}
}

void Check_RealTime(void)
{
	if (chk_real)
		chk_real = NOK;
	else
	{
		chk_real = YOK;
		printf("\nREAL Time Errpr");
	}

	//	DAQmxWriteAnalogScalarF64(AO_R, 1, 5.0, chk_real, NULL);
}

void CheckTime(void)
{
	StartTime = GetWindowTime();

	Time = SAMPLING_TIME * Tcount;
	bufTime[Tcount] = Time;

	Tcount++;
}

double GetWindowTime(void)
{
	LARGE_INTEGER   IiEndCounter, IiFrequency;
	QueryPerformanceCounter(&IiEndCounter);
	QueryPerformanceFrequency(&IiFrequency);
	return(IiEndCounter.QuadPart / (double)(IiFrequency.QuadPart) * 1000.0);
}

void IdleTime(void)
{
	while (((EndTime = GetWindowTime()) - StartTime) <= (SAMPLING_TIME*1000.0));
	if ((EndTime - StartTime) > SAMPLING_TIME*1000.0)
		chk_real = YOK;
}

void Close(void)
{
	register unsigned int idx;

	DAQmxWriteAnalogScalarF64(AO_Vcmd, 1, 5, 0, NULL);
	DAQmxWriteDigitalScalarU32(DO_Dir, 1, 1 / SAMPLING_FREQ, 0, NULL);

	DAQmxStopTask(AI_S);
	DAQmxClearTask(AI_S);

	DAQmxStopTask(AO_Vcmd);
	DAQmxClearTask(AO_Vcmd);

	DAQmxStopTask(DO_Dir);
	DAQmxClearTask(DO_Dir);

	DAQmxResetDevice("Dev1");

	// Calibration
	for (idx = START_AVG_STEP; idx < N_STEP; idx++)
		Vavg += buffA[idx]; 
	Vavg /= (N_STEP - START_AVG_STEP);
	printf("\nC Average: %lf[V]\n", Vavg);

	/*  Save Data  */
	sprintf(Text_Name, "FMF.txt");
	fopen_s(&pFile, Text_Name, "w");	
	for (idx = 2; idx < N_STEP; idx++)
		fprintf(pFile, "%10.4f	%10.4f	%10.4f	%10.4f	%10.4f	%10.4f	%10.4f	%10.4f\n", bufTime[idx], bufcmd[idx], buffA[idx], buffW[idx], buffX[idx], buffV[idx], bufmX[idx], bufmAng[idx]);

	sprintf(Text_Name, "OBS.txt");
	fopen_s(&pFile, Text_Name, "w");
	for (idx = 2; idx < N_STEP; idx++)
		fprintf(pFile, "%10.4f	%10.4f	%10.4f	%10.4f	%10.4f	%10.4f	%10.4f	%10.4f\n", bufTime[idx], bufcmd[idx], bufeA[idx], bufeW[idx], bufeX[idx], bufeV[idx], bufmX[idx], bufmAng[idx]);

	fclose(pFile);
}

void GetMeasurement(void)
{
	DAQmxReadAnalogF64(AI_S, -1, 10.0, DAQmx_Val_GroupByChannel, getvoltage, sizeof(float64)* N_AI, NULL, NULL);

	x_In = getvoltage[0];
	g_In = getvoltage[1];
	p_In = getvoltage[2];

	mAng = DEG2RAD(VOL2DEG(p_In));
	mX = VOL2X(x_In);
}

void Controller(void)
{
	fA = X_bar_beam[0];
	fW = X_bar_beam[1];
	fX = X_bar_ball[0];
	fV = X_bar_ball[1];

	eA = X_hat_curr[0]; 
	eW = X_hat_curr[1];
	eX = X_hat_curr[2];
	eV = X_hat_curr[3];

	LQServo();
	//Nbar();
}

void LQServo(void)
{
	u = prev_u - ((x_cmd - mX) + (prev_x_cmd - prev_mX))*SAMPLING_TIME*Ka2/2;
	Vc = u - (Ka11*(fA) + Ka12*(fW) + Ka13*fX + Ka14*fV);

	// Update state
	prev_u = u;
	prev_x_cmd = x_cmd;
	prev_mX = mX;
}

void Nbar(void)
{
	Vc = x_cmd / N_BAR - (K1*(fA)+K2*(fW)+K3*fX + K4*fV);
}

double Command(void)
{	
	if (Vc > V_MAX)			Vc = V_MAX;
	if (Vc < V_MIN)		Vc = V_MIN;

	if (Vc >= 0)
	{
		Motor_Dir = CCW;
		Vc_L = LINEAR_CCW(Vc);
	}
	else
	{
		Motor_Dir = CW;
		Vc_L = Vc * -1;
		Vc_L = LINEAR_CW(Vc_L);
	}

	if (Pre_Dir != Motor_Dir)
		Stop_FLAG = YOK;

	if (Stop_FLAG == YOK)
	{
		DAQmxWriteDigitalScalarU32(DO_Dir, 1, 1 / SAMPLING_FREQ, IDLE, NULL);
		Stop_Count++;

		if (Stop_Count > 3)
		{
			Stop_FLAG = NOK;
			Stop_Count = NOK;
		}
	}

	DAQmxWriteDigitalScalarU32(DO_Dir, 1, 1 / SAMPLING_FREQ, Motor_Dir, NULL);
	DAQmxWriteAnalogScalarF64(AO_Vcmd, 1, 5, Vc_L, NULL);

	// Update Dir State
	Pre_Dir = Motor_Dir;
}

void SaveData(void)
{
	bufcmd[Tcount]	= x_cmd;
	bufmV[Tcount]	= p_In; 
	bufmAng[Tcount] = mAng;
	bufmX[Tcount]	= mX;

	// FMF Data
	buffA[Tcount] = fA;
	buffW[Tcount] = fW;
	buffX[Tcount] = fX;
	buffV[Tcount] = fV;

	// Observer Data
	bufeA[Tcount] = eA;
	bufeW[Tcount] = eW;
	bufeX[Tcount] = eX;
	bufeV[Tcount] = eV;

	// Calibration
	//printf("time:%5.4f, p_In: %6.4f.\tg_In: %6.4f,\tx_In: %6.4f\n", Time, getvoltage[2], getvoltage[1], getvoltage[0]);
	
	//printf("time:%4.3f, eA:%7.4f, eW = %7.4f, eX = %5.4f, eV = %5.4f\n", Time, eA, eW, eX, eV); //X_bar_p[0]
	printf("time:%4.3f, N_bar:%4.4f fA:%7.4f, fW = %7.4f, fX = %5.4f, fV = %5.4f\n", Time, N_BAR,fA, fW, fX*100, fV); //X_bar_p[0]
}

void Observer(void)
{
	X_hat_curr[0] = X_hat_prev[0] + (X_hat_prev[1]	   - O_L1*X_hat_prev[2]					+ O_L1* mX) * SAMPLING_TIME;
	X_hat_curr[1] = X_hat_prev[1] + (A1*X_hat_prev[1]  + (A2 - O_L2)*X_hat_prev[2] + B * Vc	+ O_L2* mX) * SAMPLING_TIME;
	X_hat_curr[2] = X_hat_prev[2] + (1.0*X_hat_prev[3] - O_L3 * X_hat_prev[2]				+ O_L3* mX) * SAMPLING_TIME;
	X_hat_curr[3] = X_hat_prev[3] + (C1*X_hat_prev[0]  - O_L4*X_hat_prev[2]					+ O_L4* mX) * SAMPLING_TIME;

	X_hat_prev[0] = X_hat_curr[0];
	X_hat_prev[1] = X_hat_curr[1];
	X_hat_prev[2] = X_hat_curr[2];
	X_hat_prev[3] = X_hat_curr[3];
}

void GetFilterGain(void)
{
	/* Fading Memory Filter Gain Calulation */
	/* 2nd order  Kf = [ (1-BETA^2)    ]		3rd order	Kf = [ (1-BETA^3)                 ]   */
	/*                 [ (1-BETA)^2/Ts ]                         [ 1.5*(1-BETA)^2*(1+BETA)/Ts ]   */
	/*                                                           [ (1-BETA)^3/Ts^2            ]   */
	/* Note: BETA is inversely proportional to the filter bandwidth                               */
	/**********************************************************************************************/
	/* input	: BETA (filter design parameter subject to 0 < BETA < 1)                          */
	/* output	: Kf   (filter gain)                                                              */
	/**********************************************************************************************/

	if (N_STATE == 2)
	{
		Kf[0] = (1.0 - BETA * BETA);
		Kf[1] = (1.0 - BETA)*(1.0 - BETA) / SAMPLING_TIME;
	}
	else
	{
		Kf[0] = (1.0 - BETA * BETA * BETA);
		Kf[1] = 1.5 * (1.0 - BETA) * (1.0 - BETA) * (1.0 + BETA) / SAMPLING_TIME;
		Kf[2] = (1.0 - BETA) * (1.0 - BETA) * (1.0 - BETA) / (SAMPLING_TIME * SAMPLING_TIME);
	}
}

void MeasUpdate(void)
{
	register unsigned int idx;

	Residue_beam = (mAng - X_bar_beam[0]);
	Residue_ball = (mX - X_bar_ball[0]);

	for (idx = 0; idx < N_STATE; idx++)
	{
		X_hat_beam[idx] = X_bar_beam[idx] + Kf[idx] * Residue_beam;
		X_hat_ball[idx] = X_bar_ball[idx] + Kf[idx] * Residue_ball;
	}
}

void TimeUpdate(void)
{
	if (N_STATE == 2)
	{
		X_bar_beam[0] = X_hat_beam[0] + SAMPLING_TIME * X_hat_beam[1];
		X_bar_beam[1] = X_hat_beam[1];

		X_bar_ball[0] = X_hat_ball[0] + SAMPLING_TIME * X_hat_ball[1];
		X_bar_ball[1] = X_hat_ball[1];

	}
	else
	{
		X_bar_beam[0] = X_hat_beam[0] + SAMPLING_TIME * X_hat_beam[1] + 0.5 * (SAMPLING_TIME * SAMPLING_TIME) * X_hat_beam[2];
		X_bar_beam[1] = X_hat_beam[1] + SAMPLING_TIME * X_hat_beam[2];
		X_bar_beam[2] = X_hat_beam[2];

		X_bar_ball[0] = X_hat_ball[0] + SAMPLING_TIME * X_hat_ball[1] + 0.5 * (SAMPLING_TIME * SAMPLING_TIME) * X_hat_ball[2];
		X_bar_ball[1] = X_hat_ball[1] + SAMPLING_TIME * X_hat_ball[2];
		X_bar_ball[2] = X_hat_ball[2];
	}
}