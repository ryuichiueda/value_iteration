#ifndef VALUE_STS_
#define VALUE_STS_

#include <string>

namespace value_iteration{

class StateTransition{
public:
	int _dix, _diy, _dit, _dsigma;
	int _prob;

	StateTransition(int dix, int diy, int dit, int dsigma, int prob);
	std::string to_string(void);
};

}

#endif

