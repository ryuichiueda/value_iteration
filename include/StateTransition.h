#ifndef VALUE_STS_
#define VALUE_STS_

#include <string>
using namespace std;

class StateTransition{
public:
	int _dix, _diy, _dit;
	int _prob;

	StateTransition(int dix, int diy, int dit, int prob);
	string to_string(void);
};

#endif
