#include "value_iteration/ValueIteratorLocal.h"

namespace value_iteration{

ValueIteratorLocal::ValueIteratorLocal(std::vector<Action> &actions, int thread_num) : ValueIterator(actions, thread_num)
{
}

}
