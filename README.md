# PROJET BUS CAN  - Système Multi-Capteurs
- Récupérer sur le PC  les informations de différents capteurs. Toutes les informations vont transiter sur un bus de terrain, le bus CAN.

## Membres du groupe
- EL MAMSAOUI SOUFIANE
- MACHKOUR Ayoub
- Matane Sylvanie
- WALID Moustapha

## Architecture du système
Le système est composé de trois microcontrôleurs STM32 indépendants, chacun dédié à une fonction spécifique :

### Microcontrôleur 1 - Station météorologique (ID_1)

- Capteur VL6180X : Détection de proximité et mesure d'ambiance lumineuse
- Capteur LPS22HB : Acquisition de la pression barométrique
- Capteur HTS221 : Mesure de l'humidité relative et de la température ambiante

### Microcontrôleur 2 - Unité de mesure inertielle (ID_2)

- Accéléromètre : Détection des accélérations sur les trois axes
- Gyroscope : Mesure des vitesses de rotation
- Magnétomètre : Détection de l'orientation magnétique
- Traitement des données : Calcul des angles d'Euler (roulis, tangage, lacet)

### Microcontrôleur 3 - Contrôle d'actionneur et mesure aérodynamique (ID_3)
- Servomoteur Dynamixel : Contrôle de vitesse et de position
- Anémomètre : Mesure de la vitesse et direction du vent

## Spécifications de la communication CAN
### Attribution des identifiants
- La communication entre les nœuds utilise les identifiants suivants :
### Messages de commande :

- 0x01 - Instructions pour la station météorologique
- 0x02 - Instructions pour l'unité inertielle
- 0x03 - Instructions pour le contrôle moteur/anémomètre

### Trames de données :

- 0x22 - Données de luminosité ambiante (VL6180X)
- 0x23 - Données de proximité (VL6180X)
- 0x24 - Données barométriques (LPS22HB)
- 0x25 - Données de température (HTS221)
- 0x26 - Données d'humidité (HTS221)
- 0x95 - Données d'orientation spatiale (MPU9250)
- 0x85 - Données aérodynamiques (anémomètre)
