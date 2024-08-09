#include "motor_control.h"

// Global variables
uint32_t globalHeartbeat_50us = 0;
int measuredSpeed = 0, demandedSpeed = 0;
int demandedPWM = 0, controlOutput = 0;
uint16_t accelPedalValue_scaled = 0;

// Function to start the ADC for sampling pedals
void startADC_HALs(){
    HAL_ADC_Start(&hadc1);  // Start ADC1 for acceleration pedal
    HAL_ADC_Start(&hadc2);  // Start ADC2 for brake pedal
}

// Function to read the raw value from the brake pedal
uint16_t getRawBrakeValue(){
    return HAL_ADC_GetValue(&hadc2);
}

// Function to read the raw value from the acceleration pedal
uint16_t getRawAccelValue(){
    return HAL_ADC_GetValue(&hadc1);
}

// Scale a value between its min and max range
float scaleValue(uint16_t curr_val, uint16_t min_val, uint16_t range){
    return ((curr_val - min_val) / (float)(range));
}

// Get scaled brake pedal value
void getScaledBrakeValue(uint16_t* scaledValue, uint16_t min_val, uint16_t range){
    uint16_t rawValue = getRawBrakeValue();  // Sample raw value
    if (rawValue <= min_val) {
        (*scaledValue) = 0;
    } else {
        (*scaledValue) = (int)(4200 * scaleValue(rawValue, min_val, range));  // Scale value
    }
}

// Get scaled acceleration pedal value
void getScaledAccelValue(uint16_t* scaledValue, uint16_t min_val, uint16_t range){
    uint16_t rawValue = getRawAccelValue();  // Sample raw value
    if (rawValue <= min_val) {
        (*scaledValue) = 0;
    } else {
        (*scaledValue) = (int)(4200 * scaleValue(rawValue, min_val, range));  // Scale value
    }
    if (*scaledValue < 30) {  // Threshold to avoid accidental acceleration
        (*scaledValue) = 0;
    }
}

// Sample gear forward/backward switch
void getGearForward(bool* gearForward){
    (*gearForward) = (bool)HAL_GPIO_ReadPin(Fw_Rev_switch_GPIO_Port, Fw_Rev_switch_Pin);
}

// Initialize the array containing the 6 phases to false (motor will not move)
void initPhases(bool* Phases){
    for (int i = 0; i < 6; i++) {
        *(Phases + i) = false;
    }
}

// Initialize the array containing the hall sensor data to an arbitrary value
void initHalls(bool* Halls){
    *(Halls + 0) = true;
    *(Halls + 1) = true;
    *(Halls + 2) = false;
}

// Read all 3 Hall Sensors and store the values into the input array 'Halls'
void readHallSensors(bool* Halls){
    *(Halls + 0) = HAL_GPIO_ReadPin(GPIOD, Hall1_Pin);
    *(Halls + 1) = HAL_GPIO_ReadPin(GPIOD, Hall2_Pin);
    *(Halls + 2) = HAL_GPIO_ReadPin(GPIOD, Hall3_Pin);
}

// Compute the hall sensor position given the input sampled hall sensor data
void getHallPosition(bool Halls[3], uint8_t* hallPosition){
    (*hallPosition) = (Halls[0] << 2) + (Halls[1] << 1) + (Halls[2]);
}

// Set PWM duty cycle based on the computed phases and duty value
void setDutyCiclePWM(bool Phases[6], int dutyValue){
    TIM1->CCR1 = Phases[0] * dutyValue;  // Phase 1 High
    TIM1->CCR2 = Phases[1] * dutyValue;  // Phase 1 Low
    TIM1->CCR3 = Phases[2] * dutyValue;  // Phase 2 High
    TIM1->CCR4 = Phases[3] * dutyValue;  // Phase 2 Low
    TIM8->CCR1 = Phases[4] * dutyValue;  // Phase 3 High
    TIM8->CCR2 = Phases[5] * dutyValue;  // Phase 3 Low
}

// Apply PI control to compute the control output
void getControlOutput(int* controlOutput, int demandedSpeed, int measuredSpeed, 
                      float actuatorSaturationPoint, float* speedErrorSum, 
                      float Kp, float Ki, bool windupEnabled){
    int speedError = demandedSpeed - measuredSpeed;  // Calculate error
    (*controlOutput) = speedError * Kp + (*speedErrorSum);  // PI calculation

    float windup = 0;
    if (windupEnabled) {  // Anti-windup mechanism
        if ((*controlOutput) > actuatorSaturationPoint) {
            windup = actuatorSaturationPoint - (*controlOutput);
        } else if ((*controlOutput) < (actuatorSaturationPoint * -1)) {
            windup = (actuatorSaturationPoint * -1) - (*controlOutput);
        }
    }
    (*speedErrorSum) = (*speedErrorSum) + Ki * speedError + windup;  // Update integral error
}

int main(void){
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_TIM1_Init();
    MX_TIM3_Init();
    MX_TIM8_Init();
    MX_ADC1_Init();
    MX_ADC2_Init();

    HAL_TIM_Base_Start_IT(&htim3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_Base_Start_IT(&htim1);
    startTimerPWM();
    startADC_HALs();

    bool gearForward = true;
    bool deadManSwitch = true;
    uint8_t systemState = 0;
    uint8_t hallPosition = 0;
    uint8_t lastHallPosition = 0;

    uint16_t brakeMin_in = 1080, brakeMax_in = 2895;
    uint16_t accelMin_in = 1020, accelMax_in = 2875;
    uint16_t brakeRange = (brakeMax_in - brakeMin_in);
    uint16_t accelRange = (accelMax_in - accelMin_in);

    float motorSpeedConstant = 0.004;
    float motorBrakeConstant = 0.001;
    uint8_t supplyVoltage = 12;
    uint16_t maxMotorSpeed = 3000;

    float speedErrorSum = 0.0;
    float actuatorSaturationPoint;
    getActuatorSaturationPoint(&actuatorSaturationPoint, supplyVoltage, motorSpeedConstant);

    while (1) {
        readHallSensors(Halls);
        getHallPosition(Halls, &hallPosition);

        if (!deadManSwitch) {
            systemState = 99;
            setNullDutyCiclePWM();
        } else if (brakePedalValue_scaled > 40) {
            systemState = 0;
            setBrakingDutyCiclePWM(brakePedalValue_scaled);
            startTimerPWM();
        } else {
            if (!checkHallSensorMalfunction(hallPosition)) {
                systemState = 0;
                if (!gearForward) {
                    getPhasesReverse(Phases, hallPosition);
                } else {
                    getPhasesForward(Phases, hallPosition);
                }
                if (pidEnabled) {
                    if (demandedPWM >= 0) {
                        setDutyCiclePWM(Phases, demandedPWM);
                    } else {
                        setBrakingDutyCiclePWM(abs(demandedPWM));
                    }
                } else {
                    setDutyCiclePWM(Phases, accelPedalValue_scaled);
                }
                startTimerPWM();
            } else {
                systemState = 1;
                stopTimerPWM();
            }
        }

        // Update control variables, PID calculations, etc.

        // Perform other necessary operations like reading pedals, adjusting speed, etc.
    }
}
