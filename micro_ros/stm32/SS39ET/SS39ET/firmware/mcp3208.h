#ifndef MCP3208_H
#define MCP3208_H

#include <stdbool.h>
#include <stdint.h>

/* Met tous les chip-select à l'état haut : repos du bus SPI, aucune puce
 * sélectionnée. À appeler une fois au démarrage. */
void mcp3208_init(void);

/* Lit la tension du capteur "numero" (de 1 à 25) et la range dans "tension".
 * Renvoie true si tout s'est bien passé, false si le numéro est hors plage
 * (moins de 1 ou plus de 25) ou si la transaction SPI a échoué ; dans ce cas
 * "tension" ne doit pas être utilisée. */
bool mcp3208_lire_capteur(uint8_t numero, float *tension);

#endif /* MCP3208_H */
