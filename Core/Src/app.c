#include "main.h"
#include "drv8301.h"
#include "timebase.h"
#include <stdio.h>   // for sprintf
#include <string.h>  // for strlen
#include "debug.h"
#include "sensor.h"
#include "motor.h"
#include "foc_transforms.h"
#include <math.h>
#include "smo.h"

// Global
volatile float theta = 0.0f;
volatile float omega = 0.0f;  // rad/s

// TIM6 ISR এ (1ms প্রতি call)
// dt = 0.001s
#define DT 0.001f
#define MOTOR_R  0.09f    // ohm (per phase)
#define MOTOR_L  0.00008f // 80µH (estimated)



#define SHUNT_R        0.001f      // 1mΩ
#define OPAMP_GAIN     20.0f       // তোমার gain
#define VREF           3.3f
#define ADC_RESOLUTION 4096.0f
int32_t ia_offset_adc = 2042;
int32_t ib_offset_adc = 2042;
uint32_t forced_step_count = 0;
uint8_t  foc_active = 0;
float vd_prev = 0.0f;
float vq_prev = 0.0f;

// 6-step → electrical angle mapping
const float step_to_theta[6] = {
    0.0f,                // step 1
    1.0472f,             // step 2 → 60°
    2.0944f,             // step 3 → 120°
    3.1416f,             // step 4 → 180°
    4.1888f,             // step 5 → 240°
    5.2360f              // step 6 → 300°
};


// Global
float v_alpha_ol = 0.0f;
float v_beta_ol  = 0.0f;

// TIM6 ISR এ step এর সাথে voltage set করো
// duty = 30%, Vbus ধরো 11.1V
#define VBUS 12.0f
#define DUTY 0.30f
#define VPHASE (VBUS * DUTY)

// 6-step voltage vector (alpha-beta)
const float v_alpha_table[6] = { 1.0f,  0.5f, -0.5f, -1.0f, -0.5f,  0.5f};
const float v_beta_table[6]  = { 0.0f,  0.866f, 0.866f, 0.0f, -0.866f, -0.866f};


#define ADC_TO_AMPS(x) \
    (((float)(x) * VREF) / ((ADC_RESOLUTION - 1.0f) * SHUNT_R * OPAMP_GAIN))



DRV8301_HandleTypeDef drv;
// app.c এর উপরে add করো
extern TIM_HandleTypeDef htim6;

uint8_t current_step = 1;

#include "stm32g4xx_hal.h"

SMO_t smo;
PLL_t pll;


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

    SMO_Init(&smo,
        0.09f,      // R = 90mΩ
        0.00008f,   // L = 80µH
        0.00005f,   // dt = 50µs (20kHz)
        10.0f,      // Ksmo — tune করতে হবে
        0.4f        // Kf — LPF coefficient
    );

    PLL_Init(&pll);
//    Motor_Apply_Phase_Control(MOTOR_PHASE_A, PHASE_MODE_PWM, 100);
//    Motor_Apply_Phase_Control(MOTOR_PHASE_B, PHASE_MODE_PWM, 100);
//    Motor_Apply_Phase_Control(MOTOR_PHASE_C, PHASE_MODE_PWM, 100);

}




// ---------------- Application Main Loop ----------------
void App_Main_Loop(void)
{
//	static uint8_t step = 0;
//
//	if(Timebase_DownCounter_SS_Continuous_Expired_Event(2)){
//		step++;
//		Motor_Commutate_Step(step,50);
//		if(step>=6) step = 0;
//	}

	if(updateFlag == 1 && !foc_active){
//        int32_t theta_int  = (int32_t)(theta);
//        int32_t theta_frac = (int32_t)fabsf((smo.theta - (float)theta_int) * 1000.0f);
//        Debug_Add_Log("Theta: %ld.%03ld\n\r", theta_int, theta_frac);
//        Debug_Send_Log();
		//Sensor_ADC_Debug_Print()
		Debug_Add_Log("ADC1= %d and ADC2=%d \n\r",c_asense_adc,c_bsense_adc);
		Debug_Send_Log();
	}

    if(Timebase_DownCounter_SS_Continuous_Expired_Event(1))
    {

        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
//        Sensor_ADC_Debug_Print();
//        int32_t theta_int  = (int32_t)(smo.theta);
//        int32_t theta_frac = (int32_t)fabsf((smo.theta - (float)theta_int) * 1000.0f);
//        Debug_Add_Log("Theta: %ld.%03ld\n\r", theta_int, theta_frac);
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
    	        theta = step_to_theta[step - 1];
    	        v_alpha_ol = v_alpha_table[step-1] * VPHASE;
    	        v_beta_ol  = v_beta_table[step-1]  * VPHASE;
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
    if (hadc->Instance == ADC1){


		/* =========================
		   1. Read ADC
		========================= */
		c_asense_adc = ADC1->JDR1;
		c_bsense_adc = ADC1->JDR2;

		/* Open-loop mode → SMO/PLL OFF */
//		if (!foc_active)
//			return;

		/* =========================
		   2. ADC → Current
		========================= */
		float ia = ADC_TO_AMPS((int32_t)c_asense_adc - ia_offset_adc);
		float ib = ADC_TO_AMPS((int32_t)c_bsense_adc - ib_offset_adc);

		/* =========================
		   3. Clarke Transform
		========================= */
		Clarke_t i_clarke = Clarke(ia, ib);

		/* =========================
		   4. SMO Update
		========================= */
		SMO_Update(&smo,
				   i_clarke.alpha,
				   i_clarke.beta,
				   v_alpha_ol,
				   v_beta_ol);

		/* =========================
		   5. PLL Update (IMPORTANT ADD)
		========================= */
		PLL_Update(&pll,
				   smo.e_alpha,
				   smo.e_beta,
				   smo.dt);

		/* =========================
		   6. Final angle from PLL
		========================= */
		float theta = pll.theta;


		/* =========================
		   7. Park Transform
		========================= */
		Park_t i_park = Park(i_clarke,
							 sinf(theta),
							 cosf(theta));

		/* =========================
		   8. Control loop (future)
		========================= */
		// vd = PI_Update(...)
		// vq = PI_Update(...)

		updateFlag = 1;
    }
}


