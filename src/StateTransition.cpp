#include "value_iteration/StateTransition.h"

namespace value_iteration{

StateTransition::StateTransition(int dix, int diy, int dit, int dsigma, int prob)
{
	_dix = dix;
	_diy = diy;
	_dit = dit;
	_dsigma = dsigma;
	_prob = prob;
}

std::string StateTransition::to_string(void)
{
	return "dix:" + std::to_string(_dix) + " diy:" + std::to_string(_diy) 
		+ " dit:" + std::to_string(_dit) + " prob:" + std::to_string(_prob);
}

}
