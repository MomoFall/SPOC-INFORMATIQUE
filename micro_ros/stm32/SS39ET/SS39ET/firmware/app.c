/*
 * app.c : nœud micro-ROS "ss39et" de la carte A2.
 *
 * Il expose un seul service ROS 2, /plaque/lire_capteur, qui permet de lire
 * à la demande la tension d'un des 25 capteurs Hall :
 *   - entrée  : uint8   numero  (numéro du capteur, de 1 à 25)
 *   - sortie  : float32 tension (tension du capteur, en volts) + bool ok
 * La définition du service (srv LireCapteur) vit dans le package ROS 2
 * spoc_interfaces, cf. ros2_ws/src/spoc_interfaces de ce dépôt.
 *
 * Cycle de vie : app_init() branche micro-ROS sur l'USB, attend l'agent de
 * la Pi puis crée le service ; ensuite la boucle principale appelle
 * app_boucle() qui traite chaque requête reçue.
 */

#include <stdbool.h>

#include "app.h"
#include "main.h"
#include "mcp3208.h"

/* En-têtes micro-ROS : rcl est la couche ROS de base, rclc sa version
 * simplifiée pour microcontrôleurs (tampons pré-alloués, pas d'allocation
 * dynamique cachée). */
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>

/* Code du service LireCapteur. Il est généré dans la bibliothèque micro-ROS
 * grâce à la copie de spoc_interfaces dans extra_packages (cf. README,
 * étape 2) : c'est ce qui rend le service connu du côté STM32. */
#include <spoc_interfaces/srv/lire_capteur.h>

/* --- Adaptations matérielles de la pile micro-ROS ---
 * transport_usb.c : parle à l'agent par l'USB CDC de la Black Pill ;
 * microros_allocators.c : alloue la mémoire sans malloc. */
bool cubemx_transport_open(struct uxrCustomTransport *transport);
bool cubemx_transport_close(struct uxrCustomTransport *transport);
size_t cubemx_transport_write(struct uxrCustomTransport *transport, uint8_t *buf, size_t len, uint8_t *err);
size_t cubemx_transport_read(struct uxrCustomTransport *transport, uint8_t *buf, size_t len, int timeout, uint8_t *err);

void *microros_allocate(size_t taille, void *etat);
void  microros_deallocate(void *pointeur, void *etat);
void *microros_reallocate(void *pointeur, size_t taille, void *etat);
void *microros_zero_allocate(size_t nb_elements, size_t taille_element, void *etat);

/* --- Objets micro-ROS ---
 * Ils vivent pendant toute la durée du programme : déclarés ici et non dans
 * une fonction, sinon ils seraient détruits au retour et micro-ROS écrirait
 * dans la pile d'appels disparue. */
static rcl_node_t      noeud;
static rclc_support_t  support;
static rclc_executor_t executeur;
static rcl_service_t   service;
static spoc_interfaces__srv__LireCapteur_Request  requete;   /* dernière requête reçue */
static spoc_interfaces__srv__LireCapteur_Response reponse;   /* réponse à renvoyer */

/* Allume ou éteint la LED de la Black Pill (PC13, active à l'état bas). */
static void led(bool allumee)
{
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, allumee ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

/* Rappel exécuté par l'exécuteur à chaque appel du service depuis la Pi.
 * micro-ROS fournit la requête reçue (déjà décodée) et une réponse
 * pré-allouée qu'il ne reste qu'à remplir. */
static void rappel_lire_capteur(const void *requete_srv, void *reponse_srv, void *contexte)
{
  (void)contexte;   /* pas de contexte supplémentaire : paramètre inutilisé */

  const spoc_interfaces__srv__LireCapteur_Request *req = requete_srv;
  spoc_interfaces__srv__LireCapteur_Response      *res = reponse_srv;

  /* Lecture réelle du capteur : toute la partie SPI est dans mcp3208.c.
   * res->ok passe à false si le numéro est invalide ou si le SPI échoue ;
   * res->tension n'a alors pas de sens, et la Pi le sait grâce à ok. */
  res->ok = mcp3208_lire_capteur(req->numero, &res->tension);
}

bool app_init(void)
{
  /* 1) On remplace le transport par défaut de micro-ROS (réseau) par notre
   *    transport USB CDC : le câble USB entre la Black Pill et la Pi porte
   *    tous les échanges avec l'agent. Le "true" active le framing
   *    nécessaire à un lien série. */
  rmw_uros_set_custom_transport(
    true, NULL,
    cubemx_transport_open,
    cubemx_transport_close,
    cubemx_transport_write,
    cubemx_transport_read);

  /* 2) micro-ROS ne doit pas utiliser malloc/free : la pile prévue pour
   *    Linux n'est pas fiable sur un microcontrôleur. On lui donne nos
   *    quatre fonctions d'allocation, fournies avec le middleware. */
  rcl_allocator_t allocateur = rcutils_get_zero_initialized_allocator();
  allocateur.allocate      = microros_allocate;
  allocateur.deallocate    = microros_deallocate;
  allocateur.reallocate    = microros_reallocate;
  allocateur.zero_allocate = microros_zero_allocate;
  if (rcutils_set_default_allocator(&allocateur) != RCUTILS_RET_OK) {
    return false;   /* rien de possible sans allocateur */
  }

  /* 3) Côté matériel : ADC prêts, aucun chip-select actif. */
  mcp3208_init();

  /* 4) Connexion à l'agent micro-ROS qui tourne sur la Pi. Tant qu'il n'est
   *    pas joignable (Pi éteinte, agent pas lancé...), on réessaie toutes
   *    les secondes, LED clignotante : convention du CDC A2, section 6.5. */
  rcl_ret_t etat;
  do {
    led(true);
    HAL_Delay(500);
    led(false);
    HAL_Delay(500);
    etat = rclc_support_init(&support, 0, NULL, &allocateur);
  } while (etat != RCL_RET_OK);

  /* 5) Création du nœud : il apparaîtra dans "ros2 node list" côté Pi. */
  if (rclc_node_init_default(&noeud, "ss39et", "", &support) != RCL_RET_OK) {
    return false;
  }

  /* 6) Création du service /plaque/lire_capteur : nom aligné sur les
   *    topics /plaque/... prévus par le CDC A2. Le type vient du package
   *    spoc_interfaces, compilé dans la bibliothèque (étape 2 du README). */
  if (rclc_service_init_default(&service, &noeud,
        ROSIDL_GET_SRV_TYPE_SUPPORT(spoc_interfaces, srv, LireCapteur),
        "/plaque/lire_capteur") != RCL_RET_OK) {
    return false;
  }

  /* 7) Exécuteur : l'objet qui surveille les requêtes reçues et déclenche
   *    le rappel. Un seul objet géré (le service), et les tampons requête
   *    et réponse sont ceux déclarés plus haut : zéro allocation à chaud. */
  if (rclc_executor_init(&executeur, &support.context, 1, &allocateur) != RCL_RET_OK) {
    return false;
  }
  if (rclc_executor_add_service(&executeur, &service, &requete, &reponse,
        rappel_lire_capteur) != RCL_RET_OK) {
    return false;
  }

  /* Connecté à l'agent : LED allumée en continu (CDC A2, section 6.5). */
  led(true);
  return true;
}

void app_boucle(void)
{
  /* Traite les requêtes en attente pendant 10 ms au maximum puis rend la
   * main : la boucle principale garde la possibilité de faire autre chose
   * entre deux appels (les acquisitions périodiques du CDC viendront là). */
  (void)rclc_executor_spin_some(&executeur, RCL_MS_TO_NS(10));
}
