/* =====================================================================
 *  PROJET INDUSTRIEL : Système de Collision pour Essaim Autonome (UAV)
 *  ---------------------------------------------------------------------
 *  Encadrant: Pr. Tarik HOUICHIME
 *  École    : École des Sciences de l'Information (ESI)
 *  ---------------------------------------------------------------------
 *  OBJECTIF :
 *      Identifier en temps réel (< 1 ms) les DEUX drones les plus
 *      proches parmi N = 10 000 drones évoluant en 3D, afin de
 *      déclencher une manœuvre d'évitement avant collision.
 *
 *  CONTRAINTES STRICTES :
 *      1. Structure hétérogène : struct Drone { int id; float x,y,z; }
 *      2. Allocation dynamique unique via malloc (entrepôt continu)
 *      3. INTERDICTION FORMELLE de l'indexation par crochets []
 *         => Toute lecture/écriture se fait par arithmétique de pointeurs
 *      4. Complexité visée : O(n log n) au lieu de O(n²)
 *
 *  ALGORITHME : "Closest Pair of Points" - Diviser pour Régner
 *      - Tri préalable de l'essaim selon l'axe X        : O(n log n)
 *      - Récursion divisant l'espace en deux hémisphères : T(n)=2T(n/2)+O(n)
 *      - Master Theorem => Complexité totale : O(n log n)
 * ===================================================================== */

#include <stdio.h>      /* printf, fprintf, perror                       */
#include <stdlib.h>     /* malloc, free, qsort, exit, rand, srand        */
#include <math.h>       /* sqrtf, fabsf                                  */
#include <float.h>      /* FLT_MAX (borne supérieure pour distance)      */
#include <time.h>       /* clock_t, clock(), CLOCKS_PER_SEC, time()      */
#include <string.h>     /* memcpy (utilisé en interne par qsort)         */
#include <locale.h>     /* setlocale (pour affichage UTF-8)              */

/* ---------------------------------------------------------------------
 *  CONSTANTES DU SYSTÈME
 * --------------------------------------------------------------------- */
#define NB_DRONES        10000     /* Taille de l'essaim                 */
#define ESPACE_MAX       1000.0f   /* Cube aérien : [0, 1000]^3 mètres   */

/* ---------------------------------------------------------------------
 *  STRUCTURE HÉTÉROGÈNE IMPOSÉE PAR LE CAHIER DES CHARGES
 *  Taille mémoire : 4 (int) + 3 * 4 (float) = 16 octets par drone
 *  Total entrepôt : 16 * 10 000 = 160 000 octets = 156.25 Ko (cache-friendly)
 * --------------------------------------------------------------------- */
struct Drone {
    int   id;   /* Identifiant unique du micro-drone                     */
    float x;    /* Coordonnée spatiale X (mètres)                        */
    float y;    /* Coordonnée spatiale Y (mètres)                        */
    float z;    /* Coordonnée spatiale Z (mètres - altitude)             */
};

/* ---------------------------------------------------------------------
 *  STRUCTURE DE RÉSULTAT : la paire critique trouvée
 * --------------------------------------------------------------------- */
struct PaireCritique {
    int   id_a;        /* ID du premier drone                            */
    int   id_b;        /* ID du second drone                             */
    float distance;    /* Distance euclidienne 3D entre les deux         */
};

/* =====================================================================
 *  PARTIE 1 : FONCTIONS UTILITAIRES BAS-NIVEAU
 * ===================================================================== */

/* ---------------------------------------------------------------------
 *  distance_3d : calcul de la distance euclidienne entre deux drones
 *  Accès aux champs via DÉRÉFÉRENCEMENT de pointeur (->), JAMAIS via [].
 *  Complexité : O(1)
 * --------------------------------------------------------------------- */
static float distance_3d(const struct Drone *a, const struct Drone *b)
{
    /* Différentiels sur chaque axe spatial                              */
    float dx = a->x - b->x;
    float dy = a->y - b->y;
    float dz = a->z - b->z;

    /* Distance euclidienne : sqrt((dx)^2 + (dy)^2 + (dz)^2)             */
    return sqrtf(dx*dx + dy*dy + dz*dz);
}

/* ---------------------------------------------------------------------
 *  Comparateurs pour qsort : tri par X, par Y, par Z.
 *  qsort est en O(n log n) en moyenne (introspection sur glibc).
 * --------------------------------------------------------------------- */
static int comparer_par_x(const void *p, const void *q)
{
    /* Cast explicite des void* en pointeurs Drone                       */
    const struct Drone *d1 = (const struct Drone *)p;
    const struct Drone *d2 = (const struct Drone *)q;

    if (d1->x < d2->x) return -1;
    if (d1->x > d2->x) return  1;
    return 0;
}

static int comparer_par_y(const void *p, const void *q)
{
    const struct Drone *d1 = (const struct Drone *)p;
    const struct Drone *d2 = (const struct Drone *)q;

    if (d1->y < d2->y) return -1;
    if (d1->y > d2->y) return  1;
    return 0;
}

/* =====================================================================
 *  PARTIE 2 : ALGORITHME NAÏF (référence pour benchmark uniquement)
 *  Complexité : O(n^2) - démontre la saturation matérielle évoquée
 *  dans le cahier des charges (50 millions de calculs / ms).
 * ===================================================================== */
static struct PaireCritique recherche_naive(struct Drone *essaim, int n)
{
    struct PaireCritique res;
    res.distance = FLT_MAX;
    res.id_a = res.id_b = -1;

    /* Pointeurs de parcours : pi avance, pj balaie le reste             */
    struct Drone *pi, *pj;
    struct Drone *fin = essaim + n;   /* Sentinelle de fin d'entrepôt   */

    for (pi = essaim; pi < fin - 1; pi = pi + 1) {
        for (pj = pi + 1; pj < fin; pj = pj + 1) {
            /* Accès via dérefencement de pointeur, AUCUN crochet        */
            float d = distance_3d(pi, pj);
            if (d < res.distance) {
                res.distance = d;
                res.id_a = pi->id;
                res.id_b = pj->id;
            }
        }
    }
    return res;
}

/* =====================================================================
 *  PARTIE 3 : ALGORITHME OPTIMISÉ "DIVISER POUR RÉGNER"
 *
 *  Principe :
 *  ----------
 *  1. Trier l'essaim selon X (une seule fois, en pré-traitement).
 *  2. Diviser récursivement l'essaim en deux moitiés (gauche/droite).
 *  3. Calculer récursivement la distance min de chaque moitié.
 *  4. Soit delta = min(distance_gauche, distance_droite).
 *  5. Examiner la "bande centrale" de largeur 2*delta autour du
 *     plan médian : seuls quelques drones peuvent former une paire
 *     plus proche que delta (théorème géométrique).
 *  6. Dans cette bande, on trie par Y et on ne compare chaque drone
 *     qu'aux quelques voisins proches (constante prouvée).
 *
 *  Complexité : T(n) = 2 T(n/2) + O(n)  =>  O(n log n) (Master Theorem)
 * ===================================================================== */

/* Recherche par force brute sur une PETITE plage (cas de base récursion).
 * Utilisée quand n <= 3 : la récursion s'arrête.                        */
static float force_brute_petit(struct Drone *debut, int n,
                               struct PaireCritique *res)
{
    float min_local = FLT_MAX;
    struct Drone *pi, *pj;
    struct Drone *fin = debut + n;

    for (pi = debut; pi < fin - 1; pi = pi + 1) {
        for (pj = pi + 1; pj < fin; pj = pj + 1) {
            float d = distance_3d(pi, pj);
            if (d < min_local) {
                min_local = d;
                if (d < res->distance) {
                    res->distance = d;
                    res->id_a = pi->id;
                    res->id_b = pj->id;
                }
            }
        }
    }
    return min_local;
}

/* Examen de la bande centrale (drones à moins de delta du plan médian).
 * Les drones sont déjà triés par Y dans le tableau "bande".
 * Théorème : il suffit de comparer chaque drone à AU PLUS 7 voisins
 * suivants (constante issue de la géométrie 3D). Donc O(n).            */
static void examiner_bande(struct Drone *bande, int n_bande, float delta,
                           struct PaireCritique *res)
{
    struct Drone *pi, *pj;
    struct Drone *fin = bande + n_bande;

    for (pi = bande; pi < fin; pi = pi + 1) {
        /* On n'examine que les drones dont la différence en Y < delta. */
        for (pj = pi + 1; pj < fin && (pj->y - pi->y) < delta; pj = pj + 1) {
            float d = distance_3d(pi, pj);
            if (d < res->distance) {
                res->distance = d;
                res->id_a = pi->id;
                res->id_b = pj->id;
            }
        }
    }
}

/* Procédure récursive principale.
 * Pré-condition : "essaim" est trié par X dans la plage [0, n[.        */
static float closest_pair_rec(struct Drone *essaim, int n,
                              struct PaireCritique *res)
{
    /* CAS DE BASE : 3 drones ou moins → force brute immédiate           */
    if (n <= 3) {
        return force_brute_petit(essaim, n, res);
    }

    /* DIVISER : milieu géométrique de l'entrepôt                        */
    int milieu = n / 2;
    struct Drone *pivot = essaim + milieu;   /* arithmétique de pointeur */
    float x_median = pivot->x;

    /* RÉGNER : appel récursif sur les deux hémisphères                  */
    float dg = closest_pair_rec(essaim,           milieu,     res);
    float dd = closest_pair_rec(essaim + milieu,  n - milieu, res);

    /* Distance minimale provisoire des deux moitiés                     */
    float delta = (dg < dd) ? dg : dd;

    /* COMBINER : construire la "bande centrale" autour du plan x=x_median
     * On alloue un tableau temporaire de taille au plus n. On y copie
     * uniquement les drones tels que |x - x_median| < delta.            */
    struct Drone *bande = (struct Drone *)malloc((size_t)n * sizeof(struct Drone));
    if (bande == NULL) {
        fprintf(stderr, "[FATAL] Allocation bande centrale impossible.\n");
        exit(EXIT_FAILURE);
    }

    int n_bande = 0;
    struct Drone *p, *fin = essaim + n;
    for (p = essaim; p < fin; p = p + 1) {
        if (fabsf(p->x - x_median) < delta) {
            /* Copie via arithmétique de pointeurs : *(bande + n_bande)  */
            *(bande + n_bande) = *p;
            n_bande = n_bande + 1;
        }
    }

    /* Tri de la bande par Y (coût O(k log k) où k = taille bande)       */
    qsort(bande, (size_t)n_bande, sizeof(struct Drone), comparer_par_y);

    /* Examen optimisé de la bande - O(n) grâce au théorème géométrique  */
    examiner_bande(bande, n_bande, delta, res);

    free(bande);

    /* Retourne la nouvelle distance minimale globale                    */
    return (res->distance < delta) ? res->distance : delta;
}

/* Façade publique : prépare le terrain (tri par X) puis lance la récursion */
static struct PaireCritique recherche_optimisee(struct Drone *essaim, int n)
{
    struct PaireCritique res;
    res.distance = FLT_MAX;
    res.id_a = res.id_b = -1;

    /* Pré-traitement : tri par X - O(n log n)                           */
    qsort(essaim, (size_t)n, sizeof(struct Drone), comparer_par_x);

    /* Lancement de la récursion divide-and-conquer                      */
    closest_pair_rec(essaim, n, &res);

    return res;
}

/* =====================================================================
 *  PARTIE 4 : INITIALISATION DE L'ESSAIM
 *  Génère N drones aléatoirement répartis dans le cube [0, ESPACE_MAX]^3
 * ===================================================================== */
static void initialiser_essaim(struct Drone *essaim, int n)
{
    struct Drone *p;
    struct Drone *fin = essaim + n;
    int compteur_id = 0;

    for (p = essaim; p < fin; p = p + 1) {
        /* Écriture via dérefencement de pointeur, AUCUN crochet         */
        p->id = compteur_id;
        p->x  = ((float)rand() / (float)RAND_MAX) * ESPACE_MAX;
        p->y  = ((float)rand() / (float)RAND_MAX) * ESPACE_MAX;
        p->z  = ((float)rand() / (float)RAND_MAX) * ESPACE_MAX;
        compteur_id = compteur_id + 1;
    }
}

/* Recherche d'un drone par ID (parcours linéaire via pointeur).
 * Utilisée uniquement pour l'affichage final.                          */
static struct Drone *trouver_drone_par_id(struct Drone *essaim, int n, int id)
{
    struct Drone *p;
    struct Drone *fin = essaim + n;
    for (p = essaim; p < fin; p = p + 1) {
        if (p->id == id) return p;
    }
    return NULL;
}

/* =====================================================================
 *  PARTIE 5 : POINT D'ENTRÉE - DÉMONSTRATION ET BENCHMARK
 * ===================================================================== */
int main(void)
{
    printf("╔══════════════════════════════════════════════════════════════╗\n");
    printf("║   SYSTÈME DE COLLISION POUR ESSAIM AUTONOME (UAV)            ║\n");
    printf("║   Module de sécurité - Détection de paire critique           ║\n");
    printf("╚══════════════════════════════════════════════════════════════╝\n\n");

    /* Initialisation du générateur pseudo-aléatoire                     */
    srand((unsigned int)time(NULL));

    /* -----------------------------------------------------------------
     *  ALLOCATION UNIQUE DE L'ENTREPÔT MÉMOIRE (un seul malloc)
     *  -> Bloc continu de 10000 * sizeof(struct Drone) octets sur le tas
     * ----------------------------------------------------------------- */
    printf("[1/4] Allocation de l'entrepôt mémoire (heap)...\n");
    struct Drone *essaim = (struct Drone *)malloc(
        (size_t)NB_DRONES * sizeof(struct Drone)
    );
    if (essaim == NULL) {
        perror("malloc");
        return EXIT_FAILURE;
    }
    printf("      OK : %d drones x %lu octets = %lu Ko alloués.\n\n",
           NB_DRONES,
           (unsigned long)sizeof(struct Drone),
           (unsigned long)(NB_DRONES * sizeof(struct Drone)) / 1024);

    /* -----------------------------------------------------------------
     *  GÉNÉRATION DES POSITIONS 3D ALÉATOIRES
     * ----------------------------------------------------------------- */
    printf("[2/4] Initialisation des positions 3D aléatoires...\n");
    initialiser_essaim(essaim, NB_DRONES);
    printf("      OK : %d drones positionnés dans le cube [0,%.0f]^3 m.\n\n",
           NB_DRONES, ESPACE_MAX);

    /* -----------------------------------------------------------------
     *  COPIE DE TRAVAIL POUR L'ALGORITHME OPTIMISÉ
     *  (l'algorithme trie en place ; on garde l'original pour la
     *   comparaison naïve qui ne nécessite aucun ordre)
     * ----------------------------------------------------------------- */
    struct Drone *essaim_copie = (struct Drone *)malloc(
        (size_t)NB_DRONES * sizeof(struct Drone)
    );
    if (essaim_copie == NULL) { perror("malloc"); free(essaim); return EXIT_FAILURE; }

    /* Copie via arithmétique de pointeur                                */
    struct Drone *src, *dst;
    struct Drone *fin = essaim + NB_DRONES;
    for (src = essaim, dst = essaim_copie; src < fin; src = src + 1, dst = dst + 1) {
        *dst = *src;
    }

    /* -----------------------------------------------------------------
     *  BENCHMARK 1 : ALGORITHME NAÏF O(n^2)
     * ----------------------------------------------------------------- */
    printf("[3/4] Recherche NAÏVE O(n²) en cours...\n");
    clock_t t0 = clock();
    struct PaireCritique res_naive = recherche_naive(essaim, NB_DRONES);
    clock_t t1 = clock();
    double temps_naif = (double)(t1 - t0) / CLOCKS_PER_SEC;

    printf("      => Drones %d et %d, distance = %.6f m\n",
           res_naive.id_a, res_naive.id_b, res_naive.distance);
    printf("      => Temps écoulé : %.4f secondes\n\n", temps_naif);

    /* -----------------------------------------------------------------
     *  BENCHMARK 2 : ALGORITHME OPTIMISÉ O(n log n)
     * ----------------------------------------------------------------- */
    printf("[4/4] Recherche OPTIMISÉE O(n log n) en cours...\n");
    clock_t t2 = clock();
    struct PaireCritique res_opt = recherche_optimisee(essaim_copie, NB_DRONES);
    clock_t t3 = clock();
    double temps_opt = (double)(t3 - t2) / CLOCKS_PER_SEC;

    printf("      => Drones %d et %d, distance = %.6f m\n",
           res_opt.id_a, res_opt.id_b, res_opt.distance);
    printf("      => Temps écoulé : %.4f secondes\n\n", temps_opt);

    /* -----------------------------------------------------------------
     *  AFFICHAGE DES COORDONNÉES DES DEUX DRONES CRITIQUES
     * ----------------------------------------------------------------- */
    struct Drone *da = trouver_drone_par_id(essaim, NB_DRONES, res_opt.id_a);
    struct Drone *db = trouver_drone_par_id(essaim, NB_DRONES, res_opt.id_b);
    if (da != NULL && db != NULL) {
        printf("Coordonnées des deux drones critiques :\n");
        printf("  Drone #%d : (x=%.3f, y=%.3f, z=%.3f)\n", da->id, da->x, da->y, da->z);
        printf("  Drone #%d : (x=%.3f, y=%.3f, z=%.3f)\n", db->id, db->x, db->y, db->z);
    }

    /* -----------------------------------------------------------------
     *  RAPPORT DE PERFORMANCE
     * ----------------------------------------------------------------- */
    printf("\n┌──────────────────────── BILAN ────────────────────────────┐\n");
    printf("│ Naïf O(n²)       : %.4f s                               │\n", temps_naif);
    printf("│ Optimisé O(nlogn): %.4f s                               │\n", temps_opt);
    if (temps_opt > 0.0) {
        printf("│ Gain de vitesse  : x %.1f                                 │\n",
               temps_naif / temps_opt);
    }
    printf("│ Cohérence        : %s                   │\n",
           (res_naive.distance == res_opt.distance) ? "OK (mêmes distances)" : "à vérifier");
    printf("└───────────────────────────────────────────────────────────┘\n");

    /* -----------------------------------------------------------------
     *  LIBÉRATION DE LA MÉMOIRE (ANTI-FUITE)
     * ----------------------------------------------------------------- */
    free(essaim);
    free(essaim_copie);
    return EXIT_SUCCESS;
}