// Test HOTE de la cinematique Ackermann : aucune dependance ESP-IDF, on
// compile ackermann.c tel quel contre un faux config.h.
//
//     ./test/run.sh
//
// Ce que ce test protege, et pourquoi ca valait un fichier :
//   - le SIGNE de STEER_X_M. Une roue directrice a l'arriere fait tourner le
//     robot du cote OPPOSE. Le code d'origine supposait "devant" en dur ; sur
//     ackerbot_p4 la roue est a -0,31 m et le robot partait du mauvais cote.
//     La reference est ici la contrainte de NON-GLISSEMENT, calculee
//     independamment du modele, pas le modele lui-meme.
//   - la marche arriere, qui ne doit PAS recevoir de correction de signe.
//   - le refus propre de la rotation sur place, et le pre-braquage du bon cote.
//   - la coherence entre le differentiel electronique et l'odometrie.

#include <math.h>
#include <stdbool.h>
#include <stdio.h>

#include "../ackermann.c"

static int echecs;
static int total;

static void verifie(const char *quoi, float obtenu, float attendu, float tol)
{
    total++;
    float e = fabsf(obtenu - attendu);
    if (!(e <= tol)) {
        echecs++;
        printf("  ECHEC  %-52s obtenu %+.6f, attendu %+.6f (ecart %.2e)\n",
               quoi, (double)obtenu, (double)attendu, (double)e);
    } else {
        printf("  ok     %-52s %+.6f\n", quoi, (double)obtenu);
    }
}

static void verifie_vrai(const char *quoi, bool cond)
{
    total++;
    if (!cond) { echecs++; printf("  ECHEC  %s\n", quoi); }
    else       { printf("  ok     %s\n", quoi); }
}

// Reference INDEPENDANTE du modele : vitesse de rotation imposee par le
// non-glissement de la roue directrice, CIR sur l'axe de l'essieu moteur.
static float w_par_contrainte(float v, float xs, float delta)
{
    return v * tanf(delta) / xs;   // yc = xs/tan(delta), w = v/yc
}

int main(void)
{
    printf("Cinematique Ackermann -- STEER_X_M = %+.3f m (%s), voie %.4f m\n\n",
           (double)STEER_X_M, STEER_X_M > 0.0f ? "roue DEVANT" : "roue DERRIERE",
           (double)TRACK_WIDTH_M);

    // ---- 1. sens de rotation, compare a la contrainte de non-glissement ----
    printf("1) sens de rotation (reference = non-glissement)\n");
    const float v = 0.5f;
    for (int i = 0; i < 3; i++) {
        const float d = (float)(0.1 + 0.15 * i);
        float w_ref = w_par_contrainte(v, STEER_X_M, d);
        // le modele direct, tel que l'odometrie l'integre
        ackermann_odom_t o = {0};
        ackermann_odometry_update(&o, v * 1.0f, d, 1.0f);
        char nom[80];
        snprintf(nom, sizeof nom, "w du modele == contrainte, delta=%+.2f", (double)d);
        verifie(nom, o.w, w_ref, 1e-5f);
    }
    // et le signe est bien l'oppose entre roue devant et roue derriere
    verifie_vrai("roue devant et roue derriere donnent des sens OPPOSES",
                 w_par_contrainte(v, +0.31f, 0.3f) * w_par_contrainte(v, -0.31f, 0.3f) < 0.0f);

    // ---- 2. aller-retour consigne -> braquage -> rotation obtenue ----
    printf("\n2) aller-retour (v,w) -> delta -> w\n");
    const float essais[][2] = {
        { 0.50f,  0.30f}, { 0.50f, -0.30f},
        {-0.40f,  0.25f}, {-0.40f, -0.25f},   // MARCHE ARRIERE
        { 0.20f,  0.05f}, { 0.80f,  0.10f},
    };
    for (unsigned i = 0; i < sizeof essais / sizeof essais[0]; i++) {
        float vi = essais[i][0], wi = essais[i][1];
        float vo, del;
        bool ok = ackermann_twist_to_cmd(vi, wi, 0.0f, &vo, &del);
        float w_obtenu = w_par_contrainte(vo, STEER_X_M, del);
        char nom[80];
        snprintf(nom, sizeof nom, "v=%+.2f w=%+.2f -> delta=%+.3f, w rendu", (double)vi, (double)wi, (double)del);
        verifie(nom, w_obtenu, wi, 1e-4f);
        verifie_vrai("   demande jugee realisable", ok);
    }

    // ---- 3. rotation sur place : refusee, et pre-braquage du bon cote ----
    printf("\n3) rotation sur place\n");
    float vo, del;
    bool ok = ackermann_twist_to_cmd(0.0f, 0.5f, 0.0f, &vo, &del);
    verifie_vrai("rotation sur place REFUSEE (renvoie false)", !ok);
    verifie("   traction mise a zero", vo, 0.0f, 0.0f);
    // Depuis le 04/09/2026 : l'angle courant est RECONDUIT, il n'y a plus de
    // pre-braquage a fond -- il faisait battre le servo a l'arret pour rien.
    verifie("   angle COURANT reconduit (0 ici)", del, 0.0f, 1e-9f);
    ackermann_twist_to_cmd(0.0f, 0.5f, 0.42f, &vo, &del);
    verifie("   angle courant reconduit (0,42)", del, 0.42f, 1e-9f);
    ackermann_twist_to_cmd(0.0f, -0.5f, -0.31f, &vo, &del);
    verifie("   angle courant reconduit, w<0 (-0,31)", del, -0.31f, 1e-9f);
    ok = ackermann_twist_to_cmd(0.0f, -0.5f, 0.0f, &vo, &del);
    verifie_vrai("   demande toujours jugee IMPOSSIBLE", !ok);
    ok = ackermann_twist_to_cmd(0.0f, 0.0f, 0.0f, &vo, &del);
    verifie_vrai("arret complet accepte (renvoie true)", ok);
    verifie("   roue remise droite", del, 0.0f, 0.0f);

    // ---- 4. differentiel electronique <-> odometrie ----
    // Hors saturation, deux identites doivent tenir EXACTEMENT : la moyenne des
    // roues est la vitesse d'essieu, et le cap deduit de leur difference est
    // celui du modele. La seconde est ce qui rend le detecteur de patinage de
    // kin_ackermann.c utilisable : sans glissement il ne peut pas se declencher.
    printf("\n4) differentiel electronique et odometrie\n");
    const float v_lent = 0.2f;   // assez lent pour ne jamais saturer
    for (int i = 0; i < 4; i++) {
        const float d = (float)(-0.4 + 0.25 * i);
        float vg, vd;
        ackermann_wheel_targets(v_lent, d, &vg, &vd);
        char nom[80];
        snprintf(nom, sizeof nom, "moyenne des roues == v essieu, delta=%+.2f", (double)d);
        verifie(nom, 0.5f * (vg + vd), v_lent, 1e-6f);
        snprintf(nom, sizeof nom, "cap roues == cap modele, delta=%+.2f", (double)d);
        verifie(nom, (vd - vg) / TRACK_WIDTH_M, w_par_contrainte(v_lent, STEER_X_M, d), 1e-5f);
    }
    // ligne droite : les deux roues a la meme vitesse
    {
        float vg, vd;
        ackermann_wheel_targets(v_lent, 0.0f, &vg, &vd);
        verifie("ligne droite : roue gauche == v", vg, v_lent, 1e-7f);
        verifie("ligne droite : roue droite == v", vd, v_lent, 1e-7f);
    }

    // ---- 4b. saturation : la COURBURE doit survivre ----
    // Avec la voie de robot A et 45 deg, la roue exterieure demande 174 % de la
    // vitesse d'essieu : elle sature des 57 % de MAX_SPEED_MPS. Si on laissait
    // une seule roue saturer, le RAPPORT entre les deux changerait, donc le
    // rayon : le robot ne tournerait plus assez. C'est le piege que ce bloc
    // verrouille.
    printf("\n4b) saturation a courbure constante\n");
    for (int i = 0; i < 3; i++) {
        const float d = (float)(0.3 + 0.24 * i);
        float vg0, vd0, vg1, vd1;
        ackermann_wheel_targets(v_lent, d, &vg0, &vd0);            // sans saturer
        ackermann_wheel_targets(MAX_SPEED_MPS, d, &vg1, &vd1);     // sature
        char nom[90];
        verifie_vrai("   aucune roue au-dela de la vitesse max",
                     fabsf(vg1) <= MAX_SPEED_MPS + 1e-6f && fabsf(vd1) <= MAX_SPEED_MPS + 1e-6f);
        snprintf(nom, sizeof nom, "   rapport gauche/droite conserve, delta=%+.2f", (double)d);
        verifie(nom, vg1 / vd1, vg0 / vd0, 1e-5f);
        // NE PAS supposer LAQUELLE est l'exterieure : avec la roue directrice
        // a l'arriere, k change de signe, donc pour delta > 0 c'est la roue
        // GAUCHE qui va le plus vite -- l'inverse du cas roue devant. Ce test
        // a d'ailleurs attrape cette erreur dans sa premiere version.
        const float plus_rapide = fabsf(vg1) > fabsf(vd1) ? fabsf(vg1) : fabsf(vd1);
        snprintf(nom, sizeof nom, "   une roue exactement a la butee, delta=%+.2f", (double)d);
        verifie(nom, plus_rapide, MAX_SPEED_MPS, 1e-6f);
    }

    // ---- 5. saturation et rayon minimal ----
    printf("\n5) butees\n");
    ackermann_twist_to_cmd(0.1f, 5.0f, 0.0f, &vo, &del);
    verifie_vrai("courbure impossible -> delta SATURE, v conserve",
                 fabsf(fabsf(del) - STEER_MAX_RAD) < 1e-6f && fabsf(vo - 0.1f) < 1e-6f);
    verifie("rayon de braquage minimal derive",
            ackermann_min_turning_radius(),
            fabsf(STEER_X_M) / tanf(STEER_MAX_RAD), 1e-6f);

    printf("\n%s : %d verifications, %d echec(s)\n",
           echecs ? "ECHEC" : "TOUT PASSE", total, echecs);
    return echecs ? 1 : 0;
}
