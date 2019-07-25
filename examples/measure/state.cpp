

#include "state.h"


void state::update(toggle a, toggle b, toggle c, toggle d)
{
	ruler_start = a;
	ruler_end = b;
	ruler_bottomE = c;
	ruler_bottomS = d;
}