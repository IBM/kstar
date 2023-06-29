#include "operator_permutation.h"

#include <algorithm> 
#include <cassert>
#include "../task_proxy.h"
#include "../utils/logging.h"



using namespace std;

void OperatorPermutation::_allocate() {
	to_ops_no.assign(group.get_permutation_num_operators(), -1);
	is_op_affected.assign(group.get_permutation_num_operators(), false);    
}

OperatorPermutation::OperatorPermutation(const Group &_group, const unsigned int* full_permutation)
    : group(_group) {
    _allocate();
    int length = group.get_permutation_length();
	for (int op_no = 0; op_no < group.get_permutation_num_operators(); op_no++) {
		int op_idx = length + op_no;
		int to_op_no = full_permutation[op_idx] - length;
    	set_value(op_no, to_op_no);
	}
    finalize();
}

OperatorPermutation::OperatorPermutation(const OperatorPermutation &perm, bool invert)
    : group(perm.group) {
    _allocate();
    if (invert) {
        _inverse_value_from_permutation(perm);
    } else {
        // TODO: we could use a real, default copy constructor here.
        _copy_value_from_permutation(perm);
    }
    finalize();
}

void OperatorPermutation::dump(const TaskProxy& task_proxy) const {
	for (int op_no = 0; op_no < group.get_permutation_num_operators(); op_no++) {
		if (op_no != to_ops_no[op_no]) {
			utils::g_log << task_proxy.get_operators()[op_no].get_name() << " -> " << task_proxy.get_operators()[to_ops_no[op_no]].get_name() << " ";
		}
	}
	utils::g_log << std::endl;

}


void OperatorPermutation::_inverse_value_from_permutation(const OperatorPermutation &perm) {
	for (int op_no = 0; op_no < group.get_permutation_num_operators(); op_no++) {
		set_value(perm.get_permuted_operator_no(op_no), op_no);
    }
}

void OperatorPermutation::_copy_value_from_permutation(const OperatorPermutation &perm) {
	for (int op_no = 0; op_no < group.get_permutation_num_operators(); op_no++) {
		set_value(op_no, perm.get_permuted_operator_no(op_no));
    }
}

void OperatorPermutation::set_value(int op_no, int to_op_no) {
	assert(0 <= op_no && op_no < group.get_permutation_num_operators());
	assert(0 <= to_op_no && to_op_no < group.get_permutation_num_operators());
	assert(!is_op_affected[op_no]);

	to_ops_no[op_no] = to_op_no;

    if (op_no == to_op_no)
		return;

    // No need for the check here - should be run only once for each op_no
	if (!is_op_affected[op_no]) {
		ops_affected.push_back(op_no);
		is_op_affected[op_no] = true;
	}
}



void OperatorPermutation::finalize(){
    sort(ops_affected.begin(), ops_affected.end());
}

bool OperatorPermutation::identity() const{
    return ops_affected.size() == 0;
}

