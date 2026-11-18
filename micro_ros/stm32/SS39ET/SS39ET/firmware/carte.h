#ifndef CARTE_H
#define CARTE_H

/*
 * carte.h : constantes matérielles de la carte A2 (Black Pill STM32F411 + MCP3208).
 * Tout ce qui dépend du câblage est regroupé ici : si le PCB évolue, on ne
 * modifie que ce fichier, pas le reste du code.
 * Brochage figé par le PCB, cf. CDC logiciel carte A2, section 5.1.
 */

#include "main.h"   /* définitions HAL générées par CubeMX */

/* Nombre de capteurs à effet Hall SS39ET sur la plaque (HALL1 à HALL25). */
#define NB_CAPTEURS 25

/* Tension de référence des MCP3208 : VREF est reliée au +5 V,
 * la mesure est ratiométrique (CDC A2, section 6.1). */
#define TENSION_REF 5.0f

/* Résolution du MCP3208 : 12 bits, donc des codes entre 0 et 4095. */
#define RESOLUTION_MCP3208 4096

/*
 * Chip-select des 4 MCP3208 (une broche du STM32 par puce).
 * Sorties push-pull, niveau haut au repos (CDC A2, section 5.1).
 */
#define CS1_PORT GPIOA
#define CS1_PIN  GPIO_PIN_4   /* U1 : HALL1 à HALL8   */
#define CS2_PORT GPIOB
#define CS2_PIN  GPIO_PIN_0   /* U2 : HALL9 à HALL16  */
#define CS3_PORT GPIOB
#define CS3_PIN  GPIO_PIN_1   /* U3 : HALL17 à HALL24 */
#define CS4_PORT GPIOB
#define CS4_PIN  GPIO_PIN_10  /* U4 : HALL25 (canal 0) et TH1 à TH5 */

#endif /* CARTE_H */
