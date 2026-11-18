# spoc_interfaces

Interfaces ROS 2 du projet SPOC (service LireCapteur).

## LireCapteur

Appelé sur la carte A2 pour lire la tension d'un capteur Hall SS39ET :

- requête : numero (uint8), numéro du capteur de 1 à 25 ;
- réponse : tension (float32, volts) et ok (bool, lecture réussie).

## Utilisé par

- le firmware micro-ROS de la carte A2 : micro_ros/stm32/SS39ET (le package
  est copié dans extra_packages lors de la génération de la bibliothèque) ;
- l'agent micro-ROS de la Raspberry Pi : à recompiler avec ce package dans
  le workspace, sinon l'agent ne connaît pas le service (cf. README du SS39ET,
  étape 5) ;
- tout nœud ROS 2 qui appelle le service /plaque/lire_capteur.

## Compilation (Pi, ROS 2 Jazzy)

    cd ~/spoc-informatique/ros2_ws
    colcon build --packages-select spoc_interfaces
    source install/setup.bash
