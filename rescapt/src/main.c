#include "main.h"

//###################################################################
#define VL6180X_PRESS_HUM_TEMP 0 // Capteur de pression, humidité, température (désactivé ici)
#define MPU9250 0 // Capteur IMU (désactivé ici)
#define DYN_ANEMO 1 // Anémomètre dynamique (activé ici)
//###################################################################

//====================================================================
//          CAN ACCEPTANCE FILTER (Filtrage des messages CAN)
//====================================================================
#define USE_FILTER  1
#define ID_1    0x01 // Capteurs de luminosité, distance, pression, humidité
#define ID_2    0x02 // Accéléromètre et gyroscope
#define ID_3    0x03 // Servo-moteur et anémomètre
#define ID_4    0x04 // (Non utilisé ici)
//====================================================================
// Déclaration de fonctions externes et variables globales
extern void systemClock_Config(void);
void (*rxCompleteCallback) (void);
void can_callback(void);
void anemo_callback(void);

CAN_Message      rxMsg; // Structure pour stocker les messages CAN reçus
CAN_Message      txMsg; // Structure pour stocker les messages CAN envoyés
long int        counter = 0;

uint8_t* aTxBuffer[2]; // Buffer de transmission pour CAN

extern float magCalibration[3]; // Calibration pour le magnétomètre

// Variables pour le contrôle du moteur et le mode opératoire
static volatile int motorSpeed = 0;       // Vitesse actuelle du moteur
static volatile int isMotorOn = 1;        // État ON/OFF du moteur (par défaut: ON)
static volatile int manualSpeed = 0;      // Vitesse définie manuellement
static volatile int debugCounter = 0;     // Compteur pour déboguer
volatile int anemoSpeed = 0;              // Vitesse mesurée par l'anémomètre

// Variables pour l'optimisation des transmissions CAN
static uint32_t lastAnemoSend = 0;
static int lastAnemoSpeed = -1;
static int messageCounter = 0;

// Fonction de débogage pour surveiller l'état du système
void debugSystemState() {
    debugCounter++;
    // Afficher l'état du système toutes les 10 itérations
    if (debugCounter % 10 == 0) {
        term_printf("--- DEBUG STATE ---\n\r");
        term_printf("Motor On: %s\n\r", isMotorOn ? "YES" : "NO");
        term_printf("Motor Speed: %d\n\r", motorSpeed);
        term_printf("Anemo Speed: %d km/h\n\r", anemoSpeed);
        term_printf("Manual Speed: %d\n\r", manualSpeed);
        term_printf("Debug Counter: %d\n\r", debugCounter);
        term_printf("-------------------\n\r");
    }
}

//====================================================================
// >>>>>>>>>>>>>>>>>>>>>>>>>> MAIN <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
//====================================================================
int main(void)
{
    HAL_Init(); // Initialisation de la bibliothèque HAL
    systemClock_Config(); // Configuration de l'horloge système
    SysTick_Config(HAL_RCC_GetHCLKFreq() / 1000); // Configuration du timer SysTick pour déclencher chaque 1ms
    uart2_Init(); // Initialisation de l'UART2 pour la console
    uart1_Init(); // Initialisation de l'UART1
    i2c1_Init();  // Initialisation du bus I2C1

    HAL_Delay(1000); // Attente d'une seconde au démarrage

#if DYN_ANEMO
    // Initialisation du timer pour l'anémomètre
    anemo_Timer1Init();

    // Test et initialisation du moteur Dynamixel
    dxl_LED(1, LED_ON);
    HAL_Delay(500);
    dxl_LED(1, LED_OFF);
    HAL_Delay(500);

    // Configuration du moteur en mode vitesse
    dxl_torque(1, TORQUE_OFF);
    dxl_setOperatingMode(1, VELOCITY_MODE);
    dxl_torque(1, TORQUE_ON);
#endif

    can_Init(); // Initialisation du module CAN
    can_SetFreq(CAN_BAUDRATE); // Définition du débit CAN à 500 MHz -- cf Inc/config.h
#if USE_FILTER
    can_Filter_list((ID_1<<21)|(ID_2<<5), (ID_3<<21)|(ID_4<<5), CANStandard, 0); // Configuration du filtre CAN
#else
    can_Filter_disable(); // Désactivation du filtre pour accepter tous les messages
#endif
    can_IrqInit(); // Initialisation des interruptions CAN
    can_IrqSet(&can_callback); // Définition du callback CAN

    // Timer périodique pour les mises à jour
    tickTimer_Init(200); // Initialisation d'un timer avec une période de 200 ms

    term_printf("Système initialisé. Moteur par défaut: ON\n\r");

    while (1) {
#if DYN_ANEMO
        // Lecture et calcul de la vitesse de l'anémomètre
        int anemovitesse = anemo_GetCount();
        anemoSpeed = (anemovitesse * 5) / 4; // Conversion en km/h

        // Contrôle du moteur
        if (isMotorOn) {
            // Priorité à la vitesse manuelle
            if (manualSpeed > 0) {
                motorSpeed = manualSpeed;
                dxl_setGoalVelocity(1, motorSpeed);
                term_printf("Moteur ON - Vitesse manuelle: %d\n\r", motorSpeed);
            }
            // Sinon, utiliser la vitesse de l'anémomètre
            else if (anemoSpeed > 0) {
                motorSpeed = anemoSpeed * 10; // Conversion en vitesse de servo
                if (motorSpeed > 200) motorSpeed = 200; // Limite de vitesse
                dxl_setGoalVelocity(1, motorSpeed);
                term_printf("Moteur ON - Vitesse anémomètre: %d\n\r", motorSpeed);
            }
            else {
                // Pas de vent, arrêt du moteur
                dxl_setGoalVelocity(1, 0);
                term_printf("Moteur ON - Pas de vent\n\r");
            }
        }
        else {
            // Moteur OFF : toujours arrêté
            dxl_setGoalVelocity(1, 0);
            term_printf("Moteur OFF\n\r");
        }

        // Optimisation: envoi des données anemomètre uniquement si changement significatif
        // ou périodiquement (toutes les 250ms)
        uint32_t currentTime = HAL_GetTick();
        if ((abs(anemoSpeed - lastAnemoSpeed) > 2) || (currentTime - lastAnemoSend > 250)) {
            anemo_callback();
            lastAnemoSpeed = anemoSpeed;
            lastAnemoSend = currentTime;
        }

        // Débogage de l'état du système
        debugSystemState();

        // Réinitialisation du compteur de l'anémomètre pour le prochain cycle
        anemo_ResetCount();
#endif

        // Délai pour éviter de surcharger le système
        HAL_Delay(100);
    }

    return 0;
}

//====================================================================
//          FONCTION POUR ENVOYER LES DONNÉES DE L'ANÉMOMÈTRE
//====================================================================
void anemo_callback(void)
{
    txMsg.id = 0x85; // ID pour les données d'anémomètre

    // Formatage des données: vitesse du vent
    txMsg.data[0] = (uint8_t)(anemoSpeed & 0xFF);         // LSB
    txMsg.data[1] = (uint8_t)((anemoSpeed >> 8) & 0xFF);  // MSB

    // Ajout d'un compteur de messages pour faciliter le débogage
    messageCounter++;
    txMsg.data[2] = (uint8_t)(messageCounter & 0xFF);

    txMsg.len = 3; // Augmenté à 3 pour inclure le compteur
    txMsg.format = CANStandard;
    txMsg.type = CANData;

    can_Write(txMsg); // Envoi du message
}

//====================================================================
//          CAN CALLBACK RECEPT - Traitement des messages CAN reçus
//====================================================================
void can_callback(void)
{
    CAN_Message msg_rcv;
    can_Read(&msg_rcv);

    // Débogage des messages CAN reçus
    term_printf("MESSAGE CAN RECU - ID: 0x%lx, Longueur: %d, Data[0]: 0x%x\n\r",
                msg_rcv.id, msg_rcv.len, msg_rcv.data[0]);

    // Traitement des messages CAN en fonction de l'ID
    if (msg_rcv.id == ID_3) {
#if DYN_ANEMO
        // Commande ON/OFF (0x30)
        if (msg_rcv.data[0] == 0x30) {
            isMotorOn = !isMotorOn; // Bascule entre ON et OFF
            term_printf("Moteur %s\n\r", isMotorOn ? "ON" : "OFF");

            // Action immédiate sur le moteur
            if (!isMotorOn) {
                // Si on passe en OFF, arrêter le moteur
                dxl_setGoalVelocity(1, 0);
            }
        }
        // Commande de vitesse manuelle (0x40)
        else if (msg_rcv.data[0] == 0x40) {
            manualSpeed = msg_rcv.data[1];
            term_printf("VITESSE MANUELLE DEFINIE: %d\n\r", manualSpeed);
        }
#endif
    }
}

//====================================================================
//          TIMER CALLBACK PERIOD
//====================================================================
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    // Cette fonction est appelée périodiquement par le timer
}
