#include "main.h"
#include "drv8301.h"
#include "timebase.h"
#include <stdio.h>   // for sprintf
#include <string.h>  // for strlen
#include "debug.h"
#include "sensor.h"
#include "motor.h"
//#include "foc_transforms.h"
#include <math.h>
#include "stm32g4xx_hal.h"
#include "foc_exe.h"
#include "foc_math.h"
//#include "smo.h"
//#include "svpwm.h"
//#include "pi.h"

#include "observer_ortega.h"

// Voltage vector table (6-step, normalized)
// [step-1] = {V_alpha, V_beta}
// আগের table সম্পূর্ণ ভুল ছিল — এটা দিয়ে replace করো
static const float vab_table[6][2] = {
    { 0.8660f,  0.5000f },   // step 1: A+, B-  ← Vβ sign flip
    { 0.8660f, -0.5000f },   // step 2: A+, C-  ← Vβ sign flip
    { 0.0000f, -1.0000f },   // step 3: B+, C-  ← Vβ sign flip
    {-0.8660f, -0.5000f },   // step 4: B+, A-  ← Vβ sign flip
    {-0.8660f,  0.5000f },   // step 5: C+, A-  ← Vβ sign flip
    { 0.0000f,  1.0000f },   // step 6: C+, B-  ← Vβ sign flip
};

OrtegaObs_t observer;

// Debug এর জন্য
float dbg_theta_obs   = 0.0f;
float dbg_theta_6step = 0.0f;
float dbg_omega       = 0.0f;
float dbg_psi_alpha   = 0.0f;
float dbg_psi_beta    = 0.0f;

// TIM6 ISR এ (1ms প্রতি call)
// dt = 0.001s
#define DT 0.001f
#define MOTOR_R  0.09f    // ohm (per phase)
#define MOTOR_L  0.00008f // 80µH (estimated)



#define SHUNT_R        0.001f      // 1mΩ
#define OPAMP_GAIN     20.0f       // তোমার gain
#define VREF           3.3f
#define ADC_RESOLUTION 4096.0f
#define ADC_TO_AMPS(x) \
    (((float)(x) * VREF) / ((ADC_RESOLUTION - 1.0f) * SHUNT_R * OPAMP_GAIN))
#define VBUS 12.0f
#define DUTY 0.30f
#define VPHASE (VBUS * DUTY)



int32_t ia_offset_adc = 2042;
int32_t ib_offset_adc = 2042;
uint32_t forced_step_count = 0;
uint32_t foc_active = 0;


DRV8301_HandleTypeDef drv;
// app.c এর উপরে add করো
extern TIM_HandleTypeDef htim6;

uint8_t current_step = 1;

// ---------------- Application Setup ----------------
void App_Setup(void)
{
    // Init periodic timebase (1 kHz)
    Timebase_Init(1000);
    // Enable gate driver
    HAL_GPIO_WritePin(EN_GATE_GPIO_Port, EN_GATE_Pin, GPIO_PIN_SET);
    HAL_Delay(10); // small delay to ensure DRV8301 sees EN_GATE high
    // SPI & CS pin for DRV8301
    drv.hspi = &hspi1;
    drv.CS_Port = SPI1_CS_GPIO_Port;
    drv.CS_Pin  = SPI1_CS_Pin;

    HAL_GPIO_WritePin(drv.CS_Port, drv.CS_Pin, GPIO_PIN_SET); // CS idle high

    Motor_Init();
    Motor_Start();
    DRV8301_Init(&drv);
    //DRV8301_SetCSAGain(&drv,DRV8301_CSA_GAIN_20);
    Sensor_ADC_Init();
    //Sensor_Current_Amp_Offset_Measure();
    // Align rotor
    Timebase_DownCounter_SS_Set_Securely(1, 200);
    Timebase_DownCounter_SS_Set_Securely(2, 1);
    HAL_TIM_Base_Start_IT(&htim6);
    FOC_Init();
    Ortega_Init(&observer);
//    Motor_Apply_Phase_Control(MOTOR_PHASE_A, PHASE_MODE_PWM, 100);
//    Motor_Apply_Phase_Control(MOTOR_PHASE_B, PHASE_MODE_PWM, 100);
//    Motor_Apply_Phase_Control(MOTOR_PHASE_C, PHASE_MODE_PWM, 100);

}




// ---------------- Application Main Loop ----------------
void App_Main_Loop(void)
{

	if(updateFlag == 1 && !foc_active){

		//Sensor_ADC_Debug_Print();
		updateFlag = 0;

		Debug_Send_Log();
	}

    if(Timebase_DownCounter_SS_Continuous_Expired_Event(1))
    {

        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
//        Sensor_ADC_Debug_Print();
//        Debug_Send_Log();
    }

    Timebase_Main_Loop_Executables();
}



void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM17){
        Timebase_ISR_Executables(); // user-defined ISR function
    }

    if(htim->Instance == TIM6)
    {
    	static uint8_t step = 1;

    	    if (!foc_active)
    	    {
    	        Motor_Commutate_Step(step, 30);
    	        // TIM6 ISR এ এই line add করো step update এর পরে:
    	        current_step = step;   // observer জানবে কোন step চলছে
    	        step++;
    	        if(step > 6) step = 1;

    	        forced_step_count++;

    	        // 500 step পর FOC এ switch করো
    	        if(forced_step_count >= 5000)
    	        {
    	            foc_active = 1;
    	            Motor_Apply_Phase_Control(MOTOR_PHASE_A, PHASE_MODE_PWM, 100);
    	            Motor_Apply_Phase_Control(MOTOR_PHASE_B, PHASE_MODE_OFF, 100);
    	            Motor_Apply_Phase_Control(MOTOR_PHASE_C, PHASE_MODE_OFF, 100);
    	            // TIM6 বন্ধ করো
    	            HAL_TIM_Base_Stop_IT(&htim6);
    	        }
    	    }
    }
}




void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance != ADC1) return;

    /*****************************************************
     * 1. ADC Read
     *****************************************************/
    c_asense_adc = ADC1->JDR1;
    c_bsense_adc = ADC1->JDR2;

    /*****************************************************
     * 2. Current
     *****************************************************/
    float ia = ADC_TO_AMPS((int32_t)c_asense_adc - ia_offset_adc);
    float ib = ADC_TO_AMPS((int32_t)c_bsense_adc - ib_offset_adc);

    float I_alpha, I_beta;
    foc_clarke(ia, ib, &I_alpha, &I_beta);

    /*****************************************************
     * 3. Voltage vector from current 6-step
     *    current_step is 1-based, table is 0-based
     *****************************************************/
    uint8_t idx   = (current_step - 1) % 6;
    float V_alpha = vab_table[idx][0] * VPHASE;
    float V_beta  = vab_table[idx][1] * VPHASE;

    /*****************************************************
     * 4. Ortega Observer — always run (even open-loop)
     *    Ts = ADC trigger period
     *    TIM6 = 1ms, but ADC injected কতটুকু?
     *    তোমার ADC trigger period অনুযায়ী Ts দাও
     *****************************************************/
    Ortega_Update(&observer, V_alpha, V_beta,
                              I_alpha, I_beta, 50e-6f); // 1ms = TIM6 period

    /*****************************************************
     * 5. Debug log
     *****************************************************/
    dbg_theta_obs   = observer.theta;
    dbg_theta_6step = (float)(current_step - 1) * 1.0472f; // (step-1)*60°
    dbg_omega       = observer.omega;
    dbg_psi_alpha   = observer.psi_alpha;
    dbg_psi_beta    = observer.psi_beta;
    Debug_Add_Log("step=%d | psi=%.4f,%.4f | th_obs=%.3f th_6s=%.3f\r\n",
        current_step,          // ← এটা add করো
        dbg_psi_alpha,
        dbg_psi_beta,
        dbg_theta_obs,
        dbg_theta_6step);
    updateFlag = 1;

    if (!foc_active) return;
    // future: ADC_INJECTED_ISR_EXE();
}
