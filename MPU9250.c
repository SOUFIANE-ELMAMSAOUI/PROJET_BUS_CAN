#include "main.h"
#include "MadgwickAHRS.h"
#include <math.h>

//###################################################################
#define VL6180X_PRESS_HUM_TEMP 0 // Désactivé
#define MPU9250 1 // Capteur IMU activé
#define USE_MAGNETOMETER 1 // Magnétomètre activé pour une meilleure précision
//###################################################################

//====================================================================
//          CAN ACCEPTANCE FILTER
//====================================================================
#define USE_FILTER  1
#define ID_1    0x01
#define ID_2    0x02
#define ID_3    0x03
#define ID_4    0x04
//====================================================================

// Déclarations de fonctions et variables
extern void systemClock_Config(void);
void (*rxCompleteCallback) (void);
void can_callback(void);
void mpu9250_callback(void);

CAN_Message txMsg;
CAN_Message rxMsg;

extern float magCalibration[3]; // Calibration pour le magnétomètre
extern volatile float q0, q1, q2, q3; // Quaternion du filtre Madgwick
extern volatile float beta; // Paramètre du filtre Madgwick

// Variables pour les angles d'Euler
float phi = 0.0f;   
float theta = 0.0f; 
float psi = 0.0f;   

// Valeurs filtrées des angles
float phi_filtered = 0.0f;
float theta_filtered = 0.0f;
float psi_filtered = 0.0f;
float filter_alpha = 0.1f; // Facteur de filtrage

// Gestion du temps
uint32_t lastUpdate = 0;
const float sampleFreq = 100.0f; // 100 Hz

// Variables pour vérifier la stabilité
uint32_t measurementCount = 0;
uint8_t isSensorStable = 0;

//====================================================================
// Fonction pour calculer les angles d'Euler à partir des quaternions
//====================================================================

void updateEulerAngles() {
    // Conversion des quaternions en angles d'Euler
    phi = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * 180.0f / M_PI;
    theta = asinf(2.0f * (q0 * q2 - q3 * q1)) * 180.0f / M_PI;
    psi = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * 180.0f / M_PI;

    // Filtrage des angles pour une sortie plus stable
    phi_filtered = phi_filtered * (1.0f - filter_alpha) + phi * filter_alpha;
    theta_filtered = theta_filtered * (1.0f - filter_alpha) + theta * filter_alpha;
    psi_filtered = psi_filtered * (1.0f - filter_alpha) + psi * filter_alpha;
}

//====================================================================
// >>>>>>>>>>>>>>>>>>>>>>>>>> MAIN <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
//====================================================================
int main(void)
{
    // Initialisation du système
    HAL_Init();
    systemClock_Config();
    SysTick_Config(HAL_RCC_GetHCLKFreq() / 1000);
    uart2_Init();
    uart1_Init();
    i2c1_Init();

    HAL_Delay(1000); // Délai initial

    // Initialisation du MPU9250
    mpu9250_InitMPU9250();
    mpu9250_CalibrateMPU9250();

    #if USE_MAGNETOMETER
    mpu9250_InitAK8963(magCalibration);
    #endif

    // Vérification de l'identifiant du capteur
    uint8_t response = mpu9250_WhoAmI();
    term_printf("MPU9250 ID: %d\n\r", response);

    // Initialisation CAN
    can_Init();
    can_SetFreq(CAN_BAUDRATE);
    #if USE_FILTER
    can_Filter_list((ID_1<<21)|(ID_2<<5), (ID_3<<21)|(ID_4<<5), CANStandard, 0);
    #else
    can_Filter_disable();
    #endif
    can_IrqInit();
    can_IrqSet(&can_callback);

    // Initialisation du timer
    tickTimer_Init(200);

    // Initialisation du filtre Madgwick
    beta = 0.1f; 
    q0 = 1.0f;
    q1 = q2 = q3 = 0.0f;

    term_printf("MPU9250 initialisé. Calcul des angles d'orientation...\n\r");

    // Variables temporaires pour la lecture des capteurs
    int16_t accel_raw[3] = {0};
    int16_t gyro_raw[3] = {0};
    int16_t mag_raw[3] = {0};

    // Variables pour la conversion en unités physiques
    float ax, ay, az;
    float gx, gy, gz;
    float mx, my, mz;

    lastUpdate = HAL_GetTick();

    while (1)
    {
        uint32_t now = HAL_GetTick();
        float deltaTime = (now - lastUpdate) / 1000.0f;
        lastUpdate = now;

        // Lecture des données brutes du MPU9250
        mpu9250_ReadAccelData(accel_raw);
        mpu9250_ReadGyroData(gyro_raw);

        // Conversion des données brutes en unités physiques
        ax = accel_raw[0] * 0.000244f; // Pour ±8g
        ay = accel_raw[1] * 0.000244f;
        az = accel_raw[2] * 0.000244f;

        gx = gyro_raw[0] * 0.0174533f * 0.00763f; // Pour ±1000°/s en rad/s
        gy = gyro_raw[1] * 0.0174533f * 0.00763f;
        gz = gyro_raw[2] * 0.0174533f * 0.00763f;

        #if USE_MAGNETOMETER
        mpu9250_ReadMagData(mag_raw);
        mx = mag_raw[0] * magCalibration[0];
        my = mag_raw[1] * magCalibration[1];
        mz = mag_raw[2] * magCalibration[2];

        // Mise à jour du filtre Madgwick avec magnétomètre
        MadgwickAHRSupdate(gx, gy, gz, ax, ay, az, mx, my, mz);
        #else
        // Mise à jour du filtre Madgwick sans magnétomètre
        MadgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az);
        #endif

        // Calcul des angles d'Euler
        updateEulerAngles();

        // Incrémenter le compteur de mesures
        measurementCount++;

        // Attendre quelques mesures pour stabiliser le filtre
        if (measurementCount > 50 && !isSensorStable) {
            isSensorStable = 1;
            term_printf("Filtrage stabilisé. Envoi des angles...\n\r");
        }

        // Envoi des données d'orientation via CAN si stabilisé
        if (isSensorStable) {
            mpu9250_callback();

            // Affichage périodique des angles (toutes les 100 mesures)
            if (measurementCount % 100 == 0) {
                term_printf("Angles - Phi: %.2f, Theta: %.2f, Psi: %.2f\n\r",
                            phi_filtered, theta_filtered, psi_filtered);
            }
        }

        // Délai pour maintenir la fréquence d'échantillonnage
        HAL_Delay(10); // ~100Hz
    }

    return 0;
}

//====================================================================
//          ENVOI DES DONNÉES D'ORIENTATION DU MPU9250
//====================================================================
void mpu9250_callback(void)
{
    // ID 0x95 pour les données d'orientation
    txMsg.id = 0x95;

    // Conversion des angles filtrés en entiers (multiplié par 100)
    int16_t phi_int = (int16_t)(phi_filtered * 100.0f);
    int16_t theta_int = (int16_t)(theta_filtered * 100.0f);
    int16_t psi_int = (int16_t)(psi_filtered * 100.0f);

    // Formatage des données d'angles
    txMsg.data[0] = (uint8_t)(phi_int & 0xFF);         // LSB phi
    txMsg.data[1] = (uint8_t)((phi_int >> 8) & 0xFF);  // MSB phi

    txMsg.data[2] = (uint8_t)(theta_int & 0xFF);       // LSB theta
    txMsg.data[3] = (uint8_t)((theta_int >> 8) & 0xFF);// MSB theta

    txMsg.data[4] = (uint8_t)(psi_int & 0xFF);         // LSB psi
    txMsg.data[5] = (uint8_t)((psi_int >> 8) & 0xFF);  // MSB psi

    txMsg.len = 6;
    txMsg.format = CANStandard;
    txMsg.type = CANData;

    can_Write(txMsg);
}

//====================================================================
//          CAN CALLBACK RECEPT
//====================================================================
void can_callback(void)
{
    CAN_Message msg_rcv;
    can_Read(&msg_rcv);

    term_printf("MESSAGE CAN RECU - ID: 0x%lx, Longueur: %d\n\r",
                msg_rcv.id, msg_rcv.len);
}

//====================================================================
//          TIMER CALLBACK PERIOD
//====================================================================
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

}

