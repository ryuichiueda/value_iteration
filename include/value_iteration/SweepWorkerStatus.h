#ifndef VALUE_SWEEP_WS_
#define VALUE_SWEEP_WS_

namespace value_iteration{

class SweepWorkerStatus{
public: 	
	bool _finished;
	int _sweep_step;
	double _delta;
	bool cancel_;

	SweepWorkerStatus();
};

}

#endif

