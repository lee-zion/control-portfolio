#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <Windows.h>
#include <time.h>
#include "NIDAQmx.h"
#include "Sensor.h"

double linearization (double input_voltage)
{
	double output_voltage = 0.0;
	// double CCW_dead = 2.85;
	double CCW_dead = 2.8;
	// double CW_dead = 2.2;
	double CW_dead = 2.3;

	input_voltage += 2.5;
	
	if ((input_voltage >= 2.5) && (input_voltage <= 4.8) )
		output_voltage = (input_voltage - 2.5) * (4.8 - CCW_dead) / (4.8-2.5) + CCW_dead ;
	else if ((input_voltage < 2.5) && (input_voltage > 0.2))
		output_voltage = (input_voltage - 2.5) * (0.2 - CW_dead) / (0.2-2.5)  + CW_dead;
	else if(input_voltage > 4.8)
		output_voltage = 4.8;
	else 
		output_voltage = 0.2;
	
	return output_voltage;
}



double GetWindowTime(void)  
{
	LARGE_INTEGER	liEndCounter, liFrequency;
	QueryPerformanceCounter(&liEndCounter);
	QueryPerformanceFrequency(&liFrequency);
	return (liEndCounter.QuadPart / (double)(liFrequency.QuadPart) * 1000.0);
};


#define DAQmxErrChk(functionCall) if( DAQmxFailed(error=(functionCall)) ) goto Error; else

#define   SAMPLING_FREQ		(double)(                       200)
#define	  FINAL_TIME		(double)(		                 15)
#define   SAMPLING_TIME     (double)(           1/SAMPLING_FREQ)
#define   PI			    (double)(          3.14159265358979)
#define   ON                (int)	(			              5)
#define   OFF               (int)	(			            0.1)
#define   N_RESET			(int)   (           3*SAMPLING_FREQ)

// #define   CCW2V				(double)(			      280 )
// #define   CW2V				(double)(	       		  240 )
#define   CCW2V				(double)(			      260 )
#define   CW2V				(double)(	       		  260 )
#define   N_STEP			(int)   ( FINAL_TIME*SAMPLING_FREQ)		 
#define	  MA				(int)   (						20)
#define	  freq			    (double)(                       4)
#define   Kp_omega          (double)(                       0.6424)
#define   Ki_omega          (double)(                       19.4843)
#define	  deg               (double)(                        0.0   )
#define	  Psi_T             (double)(                        0.0   )
//const double    Ca_PI[2]				 = { 1.0, -1.0 };
//const double    Cb_PI[2]				 = { 1.5938, -1.4761}; // 0.6832, -0.5822
//const double    tblFreq[10]          = { 0.8, 0.8589, 0.9178, 0.9767, 1.0356, 1.0944, 1.1533, 1.2122, 1.2711, 1.33};		// Input frequency from 90 delay
//const double    tblFreq[10]          = { 1.0000,    1.0741,    1.1481,    1.2222 ,   1.2963 ,   1.3704  ,  1.4444  ,  1.5185,    1.5926  ,  1.6667};		// Input frequency from 90 delay
const double    tblFreq[10]          = {   1.2000 ,   1.2889  ,  1.3778 ,   1.4667 ,   1.5556  ,  1.6444  ,  1.7333  ,  1.8222  ,  1.9111 ,   2.0000};
//-----------------------------------------------------------------------------------------

void main(void)
{
	FILE*		pFile;
	double		time_prev    = 0.0;
	double		time_curr    = 0.0;
	double      time	     = 0.0;
	double      start_time	 = 0.0;
	double		final_time   = 0.0;
	double		sum          = 0.0;

	char		OutFileName[30] = { "" };
	char		errBuff[2048] = {'\0'};
	int32		error = 0;
	int			i     = 0;
	int			count = 0;
		
	double      Vout[2] = { 0.0 };
	double       Vin[5]	= { 0.0 };
	uInt32 Dout[2] ={0};
	float64 timeout = 10.0;
	int32 num_written =0;
		
	double*		time_step  ;
	double*		time_buf   ;
	double*		Ain		   ;    // Psig : Potentiometer
	double*     Ain2       ;    // Gyro : Gyroscope
	double*     Ain3       ;
	double*     Ain4       ;

	double*		Aout       ;  // flag : on = 5[V], off = 0[V]
	double*		Aout2      ;  // Wcmd or Psigc : W command voltage 
	double*		Vc		   ;
	double*		MAF		   ;
	double*     Terr       ; 
	double*		Werr	   ;
	double*		Wcmd	   ;
	double*     W_ref      ;
	double*	    Discriminator;
	int			flag = 0;
	int			j    = 0;
	double		Wslew       = 600.0;
	double		Vcmd        = 0.0;
	double		temp        = 0.0;
	double		RefPotentio = 0.0;
	double		Tcmd        = 0.0;
	double      Deg_Out     = 0.0;
	double		WcmdP       = 0.0;
	double		WcmdI       = 0.0;
	double		pWcmdI      = 0.0;
	double		Psi         = 0.0 ;
	TaskHandle	taskAI = 0;
	TaskHandle	taskAO = 0;
	
	time_step = (double*)calloc(sizeof(double), N_STEP+1);
	time_buf  = (double*)calloc(sizeof(double), N_STEP+1);
	Ain		  =	(double*)calloc(sizeof(double), N_STEP+1);
	Ain2      =	(double*)calloc(sizeof(double), N_STEP+1);
	Ain3	  =	(double*)calloc(sizeof(double), N_STEP+1);
	Ain4      =	(double*)calloc(sizeof(double), N_STEP+1);
	
	Aout	  =	(double*)calloc(sizeof(double), N_STEP+1);
	Aout2	  = (double*)calloc(sizeof(double), N_STEP+1);
	Vc		  = (double*)calloc(sizeof(double), N_STEP+1);
	MAF		  = (double*)calloc(sizeof(double), N_STEP+1);
	Terr      = (double*)calloc(sizeof(double), N_STEP+1);
	Werr      = (double*)calloc(sizeof(double), N_STEP+1);
	W_ref      = (double*)calloc(sizeof(double), N_STEP+1);
	Wcmd      = (double*)calloc(sizeof(double), N_STEP+1);
	Discriminator = (double*)calloc(sizeof(double), N_STEP+1);

	/*******************************************************************/
	/* Starting DAQ inout & output and Signal processing */
	
	DAQmxErrChk(DAQmxCreateTask("", &taskAI));
	DAQmxErrChk(DAQmxCreateTask("", &taskAO));
	
	DAQmxErrChk(DAQmxCreateAIVoltageChan(taskAI, "Dev11/ai0:4", "", DAQmx_Val_RSE, -10.0, 10.0, DAQmx_Val_Volts, "")); 
	DAQmxErrChk(DAQmxCreateAOVoltageChan(taskAO, "Dev11/ao0:1", "", 0.0, 5.0, DAQmx_Val_Volts, ""));  
 	DAQmxErrChk(DAQmxStartTask(taskAI));
	DAQmxErrChk(DAQmxStartTask(taskAO));
	/*******************************************************************/
	
	printf
	("=============================================================================\n");
	printf("                  Digital control system Project\n");
	printf("                                 2014.06 16                                  \n");
	printf("                   Lee Ja-Seung			   Lee Zion                          \n");
	printf("=========================================================================\n\n\n"); 
	printf("STEP 0 : Initalization of Gimbal System. \n") ;
	printf("If you have set the reference beam of Gimbal System, press any key to start.\n\n");
	
	
	count = 0;
	time_prev =  GetWindowTime();
		
	DAQmxErrChk(DAQmxReadAnalogF64 (taskAI, 1, timeout, DAQmx_Val_GroupByChannel, Vin, 5, NULL, NULL) ); 
	RefPotentio = Vin[0]; // 0 degree voltage !	
	Vout[0] = ON ;


	//*******************************************************************
	// Stabilization mode
		
	start_time = GetWindowTime();
	time_prev  = start_time;
	
	do
	{
		// check the simulation time and loop count 
		while (1)
	    {
			time_curr = GetWindowTime(); // present time

			if (time_curr - time_prev >= (SAMPLING_TIME * 1000.0))	break;  
		}
		time_prev = time_curr; // previous time update
		time_step[count] = SAMPLING_TIME * count ;
		
		// DAQ Reading 
		DAQmxErrChk( DAQmxReadAnalogF64 (taskAI, 1.0, timeout, DAQmx_Val_GroupByChannel, Vin, 5, NULL, NULL) ); 
		
		
		// control command
		
		
		// measure the state
		Ain[count]    = (Vin[0]- RefPotentio)* 64  ; // Potentiometer sensor input voltage to deg
		Ain2[count]   = (Vin[1]-1.23)  / 0.00333  ; // Gyro input voltage
		Ain3[count]   =  Vin[2] ; //DIR
		Ain4[count]   =  Vin[3] ; //DOA
		Discriminator[count] = Ultra_Table(Ain3[count], Ain4[count]); // Ultra Table
		
		// Moving average Filter
		if (count > 10)
		{
			sum += Ain2[count] - Ain2[count-10];
			MAF[count] = sum/10;
		}
		else
		{
			sum += Ain2[count] ;
			MAF[count] = sum ;
		}

		if (count >= 1)
		{
			W_ref[count] = 6*(Discriminator[count]); // 7*(15*sin(0.5*2*PI*time_step[count]) - Ain[count] );
			Werr[count] = (W_ref[count] - MAF[count]);
		    WcmdP = Kp_omega * Werr[count];
			WcmdI = pWcmdI + Ki_omega*SAMPLING_TIME/2.0*(Werr[count]+Werr[count-1]);
			Wcmd[count] = WcmdP + WcmdI;
			pWcmdI = WcmdI;
			
		}
		else
		{
			Wcmd[count] = 0.0;
		}
		
		
		// Wcmd[count] = 300*sin(2*PI*tblFreq[j]*time_step[count]);
		Vcmd      = Wcmd[count] ; 
		
	    if (Vcmd > 0)
			Vcmd /= CCW2V;
		else
			Vcmd /= CW2V;
		
		// linearization
		Vc[count] = linearization(Vcmd);
		
		// write the data
		Vout[1] = Vc[count] ;
		
		if (count > 10)
			DAQmxErrChk(DAQmxWriteAnalogF64(taskAO, 1,0 , timeout, DAQmx_Val_GroupByChannel, Vout, &num_written, NULL)); 
		
		
		  printf("Time[sec] : %lf Ultra %lf   Deg : %lf\n",  time_step[count] , Discriminator[count], Ain[count] );
		
	} while( count++ < (N_STEP-1)); 
	
	final_time = GetWindowTime();

	sprintf(OutFileName, "%d", j);
	pFile = fopen(strcat(OutFileName, ".out.txt"), "w+t");
	for (i = 0; i < N_STEP-1; i++)
	{
		fprintf(pFile, "%20.10lf %20.10lf %20.10lf\n", time_step[i], Discriminator[i],  Ain[i]);
		MAF[i] = 0.0;
		Wcmd[i] = 0.0;
		sum = 0.0;
	}
	fclose(pFile);

	// Reset and print the data

		count = 0;
		do {
			while (1)
			{
				time_curr = GetWindowTime();
				if (time_curr - time_prev >= (SAMPLING_TIME * 1000.0)) break;
			}
			time_prev = time_curr;
		
			/* DAQ Reading */
			DAQmxErrChk( DAQmxReadAnalogF64 (taskAI, 1, timeout, DAQmx_Val_GroupByChannel, Vin, 5, NULL, NULL) );

			Vout[1] = linearization ( 2* ( RefPotentio - Vin[0]) ) ; 
			DAQmxErrChk(DAQmxWriteAnalogF64(taskAO, 1,0 , timeout, DAQmx_Val_GroupByChannel, Vout, &num_written, NULL));
		 }while(count++ < N_RESET);
	  
		
 
	//stop the motor at the end of the system;

	Vout[0] = OFF;
	Vout[1] = 2.5;
	DAQmxErrChk(DAQmxWriteAnalogF64(taskAO, 1,0 , timeout, DAQmx_Val_GroupByChannel, Vout, &num_written, NULL)); // 
	printf("Average Real time : %lf [ms]\n",(final_time - start_time)/N_STEP);
	
	// free memory
	free(time_step) ;
	free(time_buf)  ;
	free(Ain)		;
	free(Ain2)      ;
	free(Ain3)		;
	free(Ain4)      ;
	free(Aout)		;
	free(Aout2)		;
	free(Vc)		;
	free(Terr)      ;
	free(Werr)      ;
	free(Wcmd)      ;
	free(W_ref)     ;
	free(MAF)       ;
	free(Discriminator);

	DAQmxErrChk(DAQmxStopTask(taskAI));
	DAQmxErrChk(DAQmxStopTask(taskAO));
	DAQmxErrChk(DAQmxClearTask(taskAI));
    DAQmxErrChk(DAQmxClearTask(taskAO));

Error:
	if (DAQmxFailed(error))
		DAQmxGetExtendedErrorInfo(errBuff, 2048);
	if (taskAI != 0) {
			DAQmxStopTask(taskAI);
			DAQmxClearTask(taskAI);
	}

	if (DAQmxFailed(error))
		printf("DAQmx Error: %s\n",errBuff);
		printf("End of program, press Enter key to Quit\n");
		getchar();
	}