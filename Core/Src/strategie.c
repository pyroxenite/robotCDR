/*
 * strategie.c
 *
 * "test"
 *
 * Generated by Ares ENSEA Simulator.
 * https://ares.asso-ensea.fr/simulateur/
 * 
 * Created at 00:29 on 21/09/2021.
 */

#include <strategie.h>

int nb_points = 2;

CONSIGNE consigneBleu[2] = {
    {
      .x = 0, .y = 2000, .dir = BACKWARD,
      .pinces = { FALSE, FALSE, FALSE, FALSE },
      .bras = FALSE
    }, {
      .x = 300*3, .y = 2000, .dir = FORWARD,
      .pinces = { TRUE, FALSE, FALSE, FALSE },
      .bras = FALSE
    }
};

CONSIGNE consigneJaune[7] = {
    {
      .x = 3000, .y = 2000, .dir = FORWARD,
      .pinces = { FALSE, FALSE, FALSE, FALSE },
      .bras = FALSE
    }, {
      .x = 2600, .y = 2000, .dir = BACKWARD,
      .pinces = { FALSE, FALSE, FALSE, FALSE },
      .bras = FALSE
    }, {
      .x = 2600, .y = 1600, .dir = BACKWARD,
      .pinces = { FALSE, FALSE, FALSE, FALSE },
      .bras = FALSE
    }, {
      .x = 2600, .y = 2000, .dir = BACKWARD,
      .pinces = { FALSE, FALSE, FALSE, FALSE },
      .bras = FALSE
    }, {
      .x = 3000, .y = 2000, .dir = FORWARD,
      .pinces = { FALSE, FALSE, FALSE, FALSE },
      .bras = FALSE
    }
};
