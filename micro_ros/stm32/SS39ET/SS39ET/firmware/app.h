#ifndef APP_H
#define APP_H

#include <stdbool.h>

/* Initialise le matériel et micro-ROS, puis attend la connexion à l'agent
 * micro-ROS de la Raspberry Pi (la LED clignote pendant l'attente, elle
 * reste allumée une fois connecté). Crée ensuite le service
 * /plaque/lire_capteur. Renvoie false en cas d'erreur irrécupérable. */
bool app_init(void);

/* Traite les requêtes du service en attente. À appeler en boucle depuis
 * main(), elle rend la main après 10 ms au plus. */
void app_boucle(void);

#endif /* APP_H */
