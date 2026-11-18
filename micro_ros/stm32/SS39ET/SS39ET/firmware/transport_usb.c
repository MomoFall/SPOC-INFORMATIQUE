/*
 * transport_usb.c : transport micro-ROS sur l'USB CDC de la Black Pill.
 *
 * Adapté de usb_cdc_transport.c du dépôt micro_ros_stm32cubemx_utils
 * (Apache-2.0, projet micro-ROS) : la version d'origine appelle FreeRTOS
 * (vTaskDelay), or ce firmware fonctionne sans RTOS, on attend avec HAL_Delay.
 *
 * Rôle : micro-ROS a besoin de quatre fonctions (ouvrir, fermer, lire,
 * écrire) pour parler à l'agent. Ici le lien est le port USB virtuel (CDC)
 * que la Raspberry Pi voit comme /dev/ttyACM0.
 *
 * Ne pas ajouter le usb_cdc_transport.c officiel au projet en plus de ce
 * fichier : les deux définissent les mêmes fonctions.
 */

#include <rmw_microros/rmw_microros.h>

#include "main.h"
#include "usbd_cdc_if.h"
#include "usbd_cdc.h"

#include <string.h>
#include <stdbool.h>

#ifdef RMW_UXRCE_TRANSPORT_CUSTOM

/* Handles USB générés par CubeMX. */
extern USBD_CDC_ItfTypeDef USBD_Interface_fops_FS;
extern USBD_HandleTypeDef hUsbDeviceFS;

/* Rappels CDC que l'on réimplémente : les versions générées par CubeMX ne
 * conviennent pas à micro-ROS, on les remplace par un tampon circulaire. */
static int8_t rappel_transmission(uint8_t *buf, uint32_t *len, uint8_t epnum);
static int8_t rappel_controle(uint8_t cmd, uint8_t *pbuf, uint16_t len);
static int8_t rappel_reception(uint8_t *buf, uint32_t *len);

/* Paramètres série factices renvoyés à la Pi (sans effet sur l'USB,
 * qui n'a pas de vraie vitesse en bauds). */
static uint8_t line_coding[7] = {0x00, 0xC2, 0x01, 0x00, 0x00, 0x00, 0x08};

/* Tampon circulaire de réception : l'USB écrit par interruption,
 * micro-ROS lit quand il en a besoin. */
#define TAILLE_TAMPON 2048
#define DELAI_ECRITURE_MS 100U

static volatile uint8_t tampon[TAILLE_TAMPON] = {0};
static volatile size_t tete = 0;    /* index de lecture  */
static volatile size_t queue = 0;  /* index d'écriture */
static volatile bool ecriture_finie = false;
static bool initialise = false;

/* Fin de transmission USB : on note juste que le paquet est bien parti. */
static int8_t rappel_transmission(uint8_t *buf, uint32_t *len, uint8_t epnum)
{
  (void)buf; (void)len; (void)epnum;
  ecriture_finie = true;
  return USBD_OK;
}

/* Requêtes de configuration envoyées par la Pi : on accepte tout, seul
 * le line coding est mémorisé pour être rejoué si elle le redemande. */
static int8_t rappel_controle(uint8_t cmd, uint8_t *pbuf, uint16_t len)
{
  (void)len;
  switch (cmd)
  {
    case 7:   /* CDC_SET_LINE_CODING */
      memcpy(line_coding, pbuf, sizeof(line_coding));
      break;

    case 8:   /* CDC_GET_LINE_CODING */
      memcpy(pbuf, line_coding, sizeof(line_coding));
      break;

    default:
      /* Les autres requêtes (état de ligne, break...) sont ignorées :
       * elles ne servent à rien pour micro-ROS. */
      break;
  }
  return USBD_OK;
}

/* Données reçues de la Pi : stockage dans le tampon circulaire, puis on
 * réarme la réception USB pour le paquet suivant. */
static int8_t rappel_reception(uint8_t *buf, uint32_t *len)
{
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &buf[0]);

  if ((queue + *len) > TAILLE_TAMPON) {
    /* Le paquet déborde de la fin du tampon : écriture en deux morceaux
     * pour rester circulaire (fin du tampon, puis retour au début). */
    size_t morceau1 = TAILLE_TAMPON - queue;
    size_t morceau2 = *len - morceau1;
    memcpy((void *)&tampon[queue], buf, morceau1);
    memcpy((void *)&tampon[0], buf + morceau1, morceau2);
    queue = morceau2;
  } else {
    memcpy((void *)&tampon[queue], buf, *len);
    queue += *len;
  }

  USBD_CDC_ReceivePacket(&hUsbDeviceFS);
  return USBD_OK;
}

bool cubemx_transport_open(struct uxrCustomTransport *transport)
{
  (void)transport;
  if (!initialise) {
    /* CubeMX a déjà démarré l'USB ; on remplace seulement ses rappels CDC
     * par les nôtres pour y brancher micro-ROS (une seule fois). */
    USBD_Interface_fops_FS.Control = rappel_controle;
    USBD_Interface_fops_FS.Receive = rappel_reception;
    USBD_Interface_fops_FS.TransmitCplt = rappel_transmission;
    initialise = true;
  }
  return true;
}

bool cubemx_transport_close(struct uxrCustomTransport *transport)
{
  (void)transport;
  return true;   /* rien à libérer : le CDC reste ouvert */
}

/* Écriture vers la Pi : envoi des octets, puis attente de la fin de
 * transmission (100 ms au plus) pour ne pas submerger l'USB. */
size_t cubemx_transport_write(struct uxrCustomTransport *transport, uint8_t *buf, size_t len, uint8_t *err)
{
  (void)transport;
  *err = 0;
  ecriture_finie = false;

  if (CDC_Transmit_FS(buf, (uint16_t)len) != USBD_OK) {
    *err = 1;
    return 0;
  }

  int64_t debut = uxr_millis();
  while (!ecriture_finie && ((uxr_millis() - debut) < DELAI_ECRITURE_MS)) {
    HAL_Delay(1);   /* on patiente sans RTOS : 1 ms par pas */
  }

  return ecriture_finie ? len : 0;
}

/* Lecture : on vide le tampon circulaire ; s'il est vide, on attend
 * jusqu'à "timeout" ms qu'un paquet arrive de la Pi. */
size_t cubemx_transport_read(struct uxrCustomTransport *transport, uint8_t *buf, size_t len, int timeout, uint8_t *err)
{
  (void)transport;
  *err = 0;
  size_t lus = 0;
  int64_t debut = uxr_millis();

  do {
    if (tete != queue) {
      /* Il y a des données : on les copie jusqu'à satisfaire la demande. */
      while ((tete != queue) && (lus < len)) {
        buf[lus] = tampon[tete];
        tete = (tete + 1) % TAILLE_TAMPON;
        lus++;
      }
      break;
    }
    HAL_Delay(1);   /* rien pour l'instant : on patiente 1 ms */
  } while ((uxr_millis() - debut) < timeout);

  return lus;
}

#endif /* RMW_UXRCE_TRANSPORT_CUSTOM */
