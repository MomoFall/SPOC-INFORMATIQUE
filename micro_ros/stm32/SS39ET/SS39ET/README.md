# SS39ET – lecture d'un capteur Hall via un service micro-ROS

Firmware de la carte A2 (Black Pill STM32F411CEU6) qui lit la tension des 25 capteurs à effet Hall SS39ET à travers les 4 MCP3208 reliés au SPI1, et la rend disponible dans ROS 2 par un service.

## Principe

Le nœud micro-ROS « ss39et » tourne sur la Black Pill : une fois l'agent lancé sur la Pi, il apparaît directement dans le graphe ROS 2. Le package ROS 2 côté Pi ne contient que la définition du service (spoc_interfaces), pas le nœud.

    Raspberry Pi (ROS 2 Jazzy)                     Black Pill (micro-ROS)
    -------------------------------------------    -------------------------------------------
    ros2 service call /plaque/lire_capteur        nœud micro-ROS « ss39et »
        |                                          service /plaque/lire_capteur
    agent micro-ROS (recompilé, étape 5)           transport_usb.c
        |
    DDS / reste du système ROS 2

- Entrée du service : numero (uint8), numéro du capteur à lire, de 1 à 25.
- Sortie : tension (float32, en volts) et ok (bool, vrai si la lecture a réussi).
- Définition du service : package spoc_interfaces, dans ros2_ws/src/spoc_interfaces.

Conversion : le MCP3208 sort un code sur 12 bits (0 à 4095) avec VREF = 5 V en ratiométrique (CDC A2, section 6.1), donc tension = code x 5 / 4096. Au repos un capteur sort environ 2,5 V (code voisin de 2048).

Répartition des capteurs (CDC A2, section 5.2) :

| Capteurs | Puce | Chip-select | Canaux |
| --- | --- | --- | --- |
| HALL1 à HALL8 | U1 | PA4 | 0 à 7 |
| HALL9 à HALL16 | U2 | PB0 | 0 à 7 |
| HALL17 à HALL24 | U3 | PB1 | 0 à 7 |
| HALL25 | U4 | PB10 | 0 |

D'où la formule du code : puce = (numero - 1) / 8 et canal = (numero - 1) % 8.

## Contenu du dossier

| Fichier | Rôle |
| --- | --- |
| firmware/carte.h | Constantes matérielles : broches des CS, nombre de capteurs, VREF |
| firmware/mcp3208.c et .h | Pilote des MCP3208 : trame SPI, extraction du code, conversion en volts |
| firmware/app.c et .h | Nœud micro-ROS, service /plaque/lire_capteur, LED d'état |
| firmware/transport_usb.c | Transport micro-ROS sur l'USB CDC, adapté pour fonctionner sans FreeRTOS |

transport_usb.c est une adaptation de usb_cdc_transport.c de micro_ros_stm32cubemx_utils (Apache-2.0) : la version officielle appelle FreeRTOS, celle-ci utilise HAL_Delay. Ne pas ajouter le fichier officiel au projet, il entrerait en conflit avec celui-ci.

## Étape 1 – créer et configurer le projet CubeIDE

1. Nouveau projet STM32CubeIDE, MCU STM32F411CEU6.
2. Horloge : quartz HSE 25 MHz de la Black Pill, puis PLL pour SYSCLK = 96 MHz et 48 MHz exact pour l'USB. Réglages : PLLM = 25, PLLN = 384, PLLP = /4 (96 MHz), PLLQ = /8 (48 MHz), APB1 = 48 MHz, APB2 = 96 MHz, latence flash 3 WS (CDC A2, section 2).
3. USB_OTG_FS : mode Device Only (PA11/PA12), VBUS sensing désactivé (la Black Pill ne l'utilise pas). Middleware USB_DEVICE : classe Communication Device Class (Virtual Port Com).
4. SPI1 : Full-Duplex Master. Mode 0 : CPOL Low, CPHA 1 edge. 8 bits, MSB first, NSS logiciel. Prescaler 128, soit 750 kHz (le CDC impose 1 MHz maximum, exigence EL-P2). MISO (PA6) : pas de pull-up.
5. Quatre GPIO en sortie push-pull pour les CS : PA4, PB0, PB1, PB10, niveau initial High (repos haut, CDC A2 section 5.1).
6. LED PC13 en sortie push-pull, niveau initial High (active à l'état bas).

## Étape 2 – intégrer micro-ROS dans le projet

1. À la racine du projet CubeIDE :

    git clone https://github.com/micro-ROS/micro_ros_stm32cubemx_utils

2. Copier ros2_ws/src/spoc_interfaces (ce dépôt) dans micro_ros_stm32cubemx_utils/microros_static_library/library_generation/extra_packages/ : c'est ce qui fait compiler le service LireCapteur dans la bibliothèque micro-ROS.
3. Project puis Properties puis C/C++ Build puis Settings :
   - MCU GCC Compiler, Preprocessor, Defined symbols : ajouter RMW_UXRCE_TRANSPORT_CUSTOM ;
   - MCU GCC Compiler, Include paths : ajouter micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros/include ;
   - MCU GCC Linker, Library search path (-L) : micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros, et dans Libraries (-l) : microros.
4. C/C++ Build puis Build Steps puis Pre-build steps (nécessite Docker) :

    docker pull microros/micro_ros_static_library_builder:jazzy && docker run --rm -v ${workspace_loc:/${ProjName}}:/project --env MICROROS_LIBRARY_FOLDER=micro_ros_stm32cubemx_utils/microros_static_library_ide microros/micro_ros_static_library_builder:jazzy

5. Ajouter aux sources du projet :
   - micro_ros_stm32cubemx_utils/extra_sources/microros_time.c
   - micro_ros_stm32cubemx_utils/extra_sources/microros_allocators.c
   - micro_ros_stm32cubemx_utils/extra_sources/custom_memory_manager.c
   - firmware/transport_usb.c (à la place du usb_cdc_transport.c officiel)
6. Copier firmware/app.c et firmware/mcp3208.c dans Core/Src, et firmware/app.h, firmware/mcp3208.h, firmware/carte.h dans Core/Inc.

## Étape 3 – brancher le firmware dans main.c

Dans les sections USER CODE de Core/Src/main.c :

    /* USER CODE BEGIN Includes */
    #include "app.h"
    /* USER CODE END Includes */

    /* USER CODE BEGIN 2 */
    if (!app_init()) {
      Error_Handler();   /* micro-ROS n'a pas pu démarrer */
    }
    /* USER CODE END 2 */

    /* USER CODE BEGIN WHILE */
    while (1) {
      app_boucle();      /* traite les requêtes du service */
    /* USER CODE END WHILE */

Augmenter la pile dans startup_stm32f411xe.s : Stack_Size EQU 0x2000 (8 Ko). La pile par défaut de 1 Ko est trop petite pour micro-ROS.

## Étape 4 – compiler et flasher

Le premier build génère libmicroros.a via Docker (connexion internet nécessaire, plusieurs minutes), puis compile le projet. Flasher via ST-Link (SWD, PA13/PA14) ou DFU. Après flash, la LED clignote : le firmware attend l'agent.

## Étape 5 – côté Raspberry Pi (ROS 2 Jazzy)

1. Installer le package d'interfaces :

    cd ~/spoc-informatique/ros2_ws
    colcon build --packages-select spoc_interfaces
    source install/setup.bash

2. Recompiler l'agent avec le service custom (obligatoire : l'agent doit connaître LireCapteur, l'image Docker officielle ne le connaît pas) :

    mkdir -p ~/agent_ws/src
    git clone -b jazzy https://github.com/micro-ROS/micro-ROS-Agent ~/agent_ws/src/micro_ros_agent
    cp -r ~/spoc-informatique/ros2_ws/src/spoc_interfaces ~/agent_ws/src/
    cd ~/agent_ws
    colcon build
    source install/setup.bash

3. Lancer l'agent :

    ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -v6

   Droits sur le port : sudo usermod -aG dialout,tty $USER puis se déconnecter et se reconnecter.

## Étape 6 – tester

    ros2 service list                                          # doit montrer /plaque/lire_capteur
    ros2 service type /plaque/lire_capteur                     # spoc_interfaces/srv/LireCapteur
    ros2 service call /plaque/lire_capteur spoc_interfaces/srv/LireCapteur "{numero: 1}"

Réponse attendue au repos : tension voisine de 2,5 et ok à true. En enfonçant le palpeur 1, la tension varie.

## Limites de cette version

- Une seule conversion par appel : pas de moyenne sur 8 échantillons ni de médiane glissante (CDC EL-F2), pas de filtrage. But de cette V0 : valider la chaîne complète capteur, ADC, SPI, USB, micro-ROS, service.
- Pas d'états ni de conversion en mm (EL-F3, EL-F4), pas de thermistances ni de ToF : voir le CDC A2 pour la suite.
- Pas de FreeRTOS : le CDC A2 prévoit 4 tâches ; cette V0 utilise une boucle simple, la migration est à prévoir.
- La LED suit seulement « attente agent » et « connecté » (CDC 6.5 : motif complet plus tard).

## Problèmes connus

- Si le compilateur refuse le troisième paramètre void *contexte de rappel_lire_capteur dans app.c : les versions anciennes de rclc n'avaient que deux paramètres. Supprimer le paramètre et le (void)contexte.
- Si une erreur de const se déclenche sur cubemx_transport_write : selon la version de la bibliothèque, la signature attend const uint8_t *buf. Ajouter le const dans transport_usb.c et app.c.
- Le port USB peut apparaître comme /dev/ttyUSB0 selon le câble ou l'adaptateur.
- Le premier build télécharge les sources micro-ROS : prévoir une connexion internet et plusieurs minutes.

## Références

- CDC logiciel carte A2 (sections 2, 5.1, 5.2, 6.1) : document interne SPOC.
- Intégration micro-ROS dans CubeIDE : https://github.com/micro-ROS/micro_ros_stm32cubemx_utils
- Agent micro-ROS : https://github.com/micro-ROS/micro-ROS-Agent
- MCP3208 : datasheet Microchip DS21298.
- Capteur : Honeywell SS39ET.
