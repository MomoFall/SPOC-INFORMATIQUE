/*
 * mcp3208.c : lecture des capteurs Hall SS39ET à travers les 4 MCP3208.
 *
 * Un MCP3208 est un ADC 12 bits à 8 canaux, commandé en SPI mode 0.
 * La carte A2 en porte 4, tous reliés au SPI1 du STM32, avec un chip-select
 * différent par puce. Les 25 capteurs sont répartis ainsi (CDC A2, 5.2) :
 *   capteurs 1 à 8    -> puce 1, canaux 0 à 7
 *   capteurs 9 à 16   -> puce 2, canaux 0 à 7
 *   capteurs 17 à 24  -> puce 3, canaux 0 à 7
 *   capteur 25        -> puce 4, canal 0
 * D'où la formule utilisée plus bas : puce = (numero - 1) / 8 et
 * canal = (numero - 1) % 8.
 */

#include "mcp3208.h"
#include "carte.h"
#include "spi.h"     /* hspi1, générée par CubeMX */

/* Garde-fou : durée maximale d'une transaction SPI en millisecondes.
 * Si l'ADC ne répond pas, on ne bloque pas le service pour toujours. */
#define DUREE_SPI_MS 10

/* Ports et broches des chip-select, dans l'ordre des puces 1 à 4.
 * Deux tableaux plutôt qu'une structure pour rester en C99 simple. */
static GPIO_TypeDef * const ports_cs[4] = { CS1_PORT, CS2_PORT, CS3_PORT, CS4_PORT };
static const uint16_t broches_cs[4]     = { CS1_PIN,  CS2_PIN,  CS3_PIN,  CS4_PIN  };

void mcp3208_init(void)
{
  /* Au repos aucune puce ne doit être sélectionnée : tous les CS à l'état
   * haut. CubeMX le fait déjà au démarrage, on le refait ici par sécurité
   * (utile aussi si on ajoute plus tard une remise à zéro du bus). */
  for (uint8_t puce = 0; puce < 4; puce++) {
    HAL_GPIO_WritePin(ports_cs[puce], broches_cs[puce], GPIO_PIN_SET);
  }
}

/* Lit un canal d'une puce et range le code brut sur 12 bits dans "code".
 * puce : 0 à 3 (U1 à U4) ; canal : 0 à 7.
 * Renvoie false si la transaction SPI a échoué. */
static bool lire_canal(uint8_t puce, uint8_t canal, uint16_t *code)
{
  /* Trame MCP3208 en mesure single-ended (CDC A2, section 5.2) :
   * - octet 0 : bit de start (1) + SGL/DIFF (1) + bit de poids fort du canal,
   *   soit 0000 0110 = 0x06 auquel on ajoute les 2 bits de poids fort du canal
   * - octet 1 : les 2 bits de poids faible du canal en positions 7 et 6
   * - octet 2 : à zéro, il laisse à l'ADC le temps de sortir le résultat */
  uint8_t emission[3]  = { (uint8_t)(0x06 | (canal >> 2)), (uint8_t)((canal & 0x03) << 6), 0x00 };
  uint8_t reception[3] = { 0, 0, 0 };

  /* CS bas : la puce sélectionnée prend le bus (une seule à la fois,
   * sinon leurs sorties se battent). */
  HAL_GPIO_WritePin(ports_cs[puce], broches_cs[puce], GPIO_PIN_RESET);

  /* Échange des 3 octets en mode 0. On note l'état pour prévenir l'appelant
   * en cas d'échec, mais on relâche le CS dans tous les cas : ne jamais
   * laisser le bus bloqué. */
  HAL_StatusTypeDef etat = HAL_SPI_TransmitReceive(&hspi1, emission, reception, 3, DUREE_SPI_MS);

  /* CS haut : fin de conversion, la puce relâche le bus. */
  HAL_GPIO_WritePin(ports_cs[puce], broches_cs[puce], GPIO_PIN_SET);

  if (etat != HAL_OK) {
    return false;
  }

  /* Le résultat arrive sur 3 octets : 1 octet nul, puis 4 bits nuls + les
   * 4 bits de poids fort du code, puis les 8 bits de poids faible.
   * On garde donc les 4 bits bas de reception[1] et tout reception[2]. */
  *code = (uint16_t)(((reception[1] & 0x0F) << 8) | reception[2]);
  return true;
}

bool mcp3208_lire_capteur(uint8_t numero, float *tension)
{
  /* Rejet des numéros impossibles : la plaque n'a que 25 capteurs. */
  if ((numero < 1) || (numero > NB_CAPTEURS)) {
    return false;
  }

  /* Position du capteur sur les ADC (cf. commentaire d'en-tête).
   * Exemple : capteur 9 -> puce (9 - 1) / 8 = 1 (U2), canal (9 - 1) % 8 = 0. */
  uint8_t puce  = (uint8_t)((numero - 1) / 8);
  uint8_t canal = (uint8_t)((numero - 1) % 8);

  uint16_t code;
  if (!lire_canal(puce, canal, &code)) {
    return false;   /* échec SPI : l'appelant saura que la valeur est invalide */
  }

  /* Conversion code -> tension. L'ADC est ratiométrique : 4096 codes
   * couvrent VREF = 5 V (CDC A2, section 6.1).
   * Exemple : code 2048 au repos -> environ 2,5 V en sortie du capteur. */
  *tension = ((float)code) * TENSION_REF / RESOLUTION_MCP3208;
  return true;
}
