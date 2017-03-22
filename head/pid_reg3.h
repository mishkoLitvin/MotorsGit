/* =================================================================================
File name:       PID_REG3.H                     
                    
Originator:	Digital Control Systems Group
			Texas Instruments

Description: 
Header file containing constants, data type definitions, and 
function prototypes for the PIDREG3.
=====================================================================================
 History:
-------------------------------------------------------------------------------------
 04-15-2005	Version 3.20
------------------------------------------------------------------------------*/

struct _PIDREG3_ {  float32  Ref;   		// Input: Reference input
				  float32  Fdb;   		// Input: Feedback input 
				  float32  Err;			// Variable: Error 
				  float32  Kp;			// Parameter: Proportional gain
				  float32  Up;			// Variable: Proportional output 
				  float32  Ui;			// Variable: Integral output 
				  float32  Ud;			// Variable: Derivative output	
				  float32  OutPreSat;	// Variable: Pre-saturated output
				  float32  OutMax;		// Parameter: Maximum output 
				  float32  OutMin;		// Parameter: Minimum output
				  float32  OutIMax;		// Parameter: Maximum integral output
				  float32  OutIMin;		// Parameter: Minimum integral output
				  float32  Out;   		// Output: PID output 
				  float32  SatErr;		// Variable: Saturated difference
				  float32  Ki;			// Parameter: Integral gain
				  float32  Kc;			// Parameter: Integral correction gain
				  float32  Kd; 			// Parameter: Derivative gain 
				  float32  Up1;			// History: Previous proportional output
		 	 	  void  (*calc)();	  	// Pointer to calculation function
				 } ;

typedef struct _PIDREG3_ PIDREG3;

typedef PIDREG3* PIDREG3_handle;

/*-----------------------------------------------------------------------------
Default initalizer for the PIDREG3 object.
-----------------------------------------------------------------------------*/                     
#define PIDREG3_DEFAULTS { 0, \
                           0, \
                           0, \
                           0.1, \
                           0, \
                           0, \
                           0, \
                           0, \
                           1., \
                           -1., \
                           0.8, \
                           -0.8, \
                           0, \
                           0, \
                           10.00, \
                           0.000, \
                           .0, \
                           0, \
              			  (void (*)(Uint32))pid_reg3_calc }

/*------------------------------------------------------------------------------
Prototypes for the functions in PIDREG3.C
------------------------------------------------------------------------------*/
void pid_reg3_calc(PIDREG3_handle);

void pid_reg3_calc(PIDREG3 *v)
{
    // Compute the error
    v->Err = v->Ref - v->Fdb;

    // Compute the proportional output
    v->Up = v->Kp*v->Err;

    // Compute the integral output
    v->Ui = v->Ui + v->Ki*v->Up + v->Kc*v->SatErr;
//    v->Ui = v->Ui + v->Ki*v->Err + v->Kc*v->SatErr;
    if(v->Ui>v->OutIMax)
    	v->Ui = v->OutIMax;
    else
    	if(v->Ui<v->OutIMin)
    	    	v->Ui = v->OutIMin;

    // Compute the derivative output
//    v->Ud = v->Kd*(v->Up - v->Up1);
    v->Ud = v->Kd*(v->Err);
    // Compute the pre-saturated output
    v->OutPreSat = v->Up + v->Ui + v->Ud;

    // Saturate the output
    if (v->OutPreSat > v->OutMax)
      v->Out =  v->OutMax;
    else if (v->OutPreSat < v->OutMin)
      v->Out =  v->OutMin;
    else
      v->Out = v->OutPreSat;

    // Compute the saturate difference
    v->SatErr = v->Out - v->OutPreSat;

    // Update the previous proportional output
    v->Up1 = v->Up;

}

