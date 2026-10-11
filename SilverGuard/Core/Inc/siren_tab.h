#ifndef SIREN_TAB_H
#define SIREN_TAB_H

#include <stdint.h>

#define SIREN_N 32
#define SIREN_T 900

static const uint16_t siren_tab[SIREN_N] = {
	555,531,509,489,471,454,438,423,
	409,396,384,372,361,351,341,332,
	332,341,351,361,372,384,396,409,
	423,438,454,471,489,509,531,555,
};

#endif
