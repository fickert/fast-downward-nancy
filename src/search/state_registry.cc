#include "state_registry.h"

#include "per_state_information.h"
#include "task_proxy.h"

#include "task_utils/task_properties.h"

// #define TRACKSTATEREG

#ifdef TRACKSTATEREG
#define BEGINF(X) std::cout << "ENTER: " << X << "\n";
#define ENDF(X) std::cout << "EXIT: " << X << "\n";
#else
#define BEGINF(X)
#define ENDF(X)
#endif


using namespace std;

StateRegistry::StateIDSemanticHash::StateIDSemanticHash(
            const segmented_vector::SegmentedArrayVector<PackedStateBin> *state_data_pool,
            int state_size)
            : state_data_pool(state_data_pool),
              state_size(state_size)
{
}

StateRegistry::StateIDSemanticHash::StateIDSemanticHash(const StateIDSemanticHash &sh)
		    :state_data_pool(sh.state_data_pool),
		     state_size(sh.state_size)
{
}


int_hash_set::HashType StateRegistry::StateIDSemanticHash::operator()(int id) const
 {
	 const PackedStateBin *data = (*state_data_pool)[id];
	 utils::HashState hash_state;
	 for (int i = 0; i < state_size; ++i) {
		 hash_state.feed(data[i]);
	 }
	 return hash_state.get_hash32();
 }

StateRegistry::StateIDSemanticHash &StateRegistry::StateIDSemanticHash::operator=(StateIDSemanticHash &&sh)
{
	state_data_pool = sh.state_data_pool;
	sh.state_data_pool = nullptr;
	state_size = std::move(sh.state_size);
	return *this;
}

StateRegistry::StateIDSemanticEqual::StateIDSemanticEqual(
            const segmented_vector::SegmentedArrayVector<PackedStateBin> *state_data_pool,
            int state_size)
            : state_data_pool(state_data_pool),
              state_size(state_size)
{
}

StateRegistry::StateIDSemanticEqual::StateIDSemanticEqual(const StateIDSemanticEqual &se)
	:state_data_pool(se.state_data_pool),
	 state_size(se.state_size)
{
}

bool StateRegistry::StateIDSemanticEqual::operator()(int lhs, int rhs) const
{
	const PackedStateBin *lhs_data = (*state_data_pool)[lhs];
	const PackedStateBin *rhs_data = (*state_data_pool)[rhs];
	return std::equal(lhs_data, lhs_data + state_size, rhs_data);
}

StateRegistry::StateIDSemanticEqual &StateRegistry::StateIDSemanticEqual::operator=(StateIDSemanticEqual &&se)
{
	state_data_pool = se.state_data_pool;
	se.state_data_pool = nullptr;
	state_size = std::move(se.state_size);
	return *this;
}

StateRegistry::StateRegistry(const TaskProxy &task_proxy)
    : task_proxy(task_proxy),
      state_packer(task_properties::g_state_packers[task_proxy]),
      axiom_evaluator(g_axiom_evaluators[task_proxy]),
      num_variables(task_proxy.get_variables().size()),
      state_data_pool(get_bins_per_state()),
      registered_states(
          StateIDSemanticHash(&state_data_pool, get_bins_per_state()),
          StateIDSemanticEqual(&state_data_pool, get_bins_per_state())),
      cached_initial_state(0) {
}

StateRegistry &StateRegistry::operator=(StateRegistry &&sr)
{
	BEGINF(__func__);
	assert(sr.registered_states.get_hasher().state_data_pool == &sr.state_data_pool);
	assert(sr.registered_states.get_equal().state_data_pool == &sr.state_data_pool);

	task_proxy = std::move(sr.task_proxy);
	const_cast<int_packer::IntPacker &>(state_packer) = std::move(sr.state_packer);
	axiom_evaluator = sr.axiom_evaluator;
	*(const_cast<int*>(&num_variables)) = std::move(sr.num_variables);
	state_data_pool = std::move(sr.state_data_pool);
	registered_states = std::move(sr.registered_states);
	// 'hasher' and 'equal' both have a pointer to the
	// state_data_pool in the registry itself, which has to be
	// updated seperately here.
	registered_states.get_hasher().state_data_pool = &state_data_pool;
	registered_states.get_equal().state_data_pool = &state_data_pool;
	cached_initial_state = std::move(sr.cached_initial_state);

	sr.cached_initial_state = nullptr;

	ENDF(__func__);
	return *this;
}

StateRegistry::~StateRegistry() {
	if (cached_initial_state != nullptr)
		delete cached_initial_state;
}

StateID StateRegistry::insert_id_or_pop_state() {
    /*
      Attempt to insert a StateID for the last state of state_data_pool
      if none is present yet. If this fails (another entry for this state
      is present), we have to remove the duplicate entry from the
      state data pool.
    */
    StateID id(state_data_pool.size() - 1);
    pair<int, bool> result = registered_states.insert(id.value);
    bool is_new_entry = result.second;
    if (!is_new_entry) {
        state_data_pool.pop_back();
    }
    assert(registered_states.size() == static_cast<int>(state_data_pool.size()));
    return StateID(result.first);
}

GlobalState StateRegistry::lookup_state(StateID id) const {
    return GlobalState(state_data_pool[id.value], *this, id);
}

const GlobalState &StateRegistry::get_initial_state() {
    if (cached_initial_state == 0) {
        PackedStateBin *buffer = new PackedStateBin[get_bins_per_state()];
        // Avoid garbage values in half-full bins.
        fill_n(buffer, get_bins_per_state(), 0);

        State initial_state = task_proxy.get_initial_state();
        for (size_t i = 0; i < initial_state.size(); ++i) {
            state_packer.set(buffer, i, initial_state[i].get_value());
        }
        state_data_pool.push_back(buffer);
        // buffer is copied by push_back
        delete[] buffer;
        StateID id = insert_id_or_pop_state();
        cached_initial_state = new GlobalState(lookup_state(id));
    }
    return *cached_initial_state;
}

//TODO it would be nice to move the actual state creation (and operator application)
//     out of the StateRegistry. This could for example be done by global functions
//     operating on state buffers (PackedStateBin *).
GlobalState StateRegistry::get_successor_state(const GlobalState &predecessor, const OperatorProxy &op) {
    assert(!op.is_axiom());
    state_data_pool.push_back(predecessor.get_packed_buffer());
    PackedStateBin *buffer = state_data_pool[state_data_pool.size() - 1];
    for (EffectProxy effect : op.get_effects()) {
        if (does_fire(effect, predecessor)) {
            FactPair effect_pair = effect.get_fact().get_pair();
            state_packer.set(buffer, effect_pair.var, effect_pair.value);
        }
    }
    axiom_evaluator.evaluate(buffer, state_packer);
    StateID id = insert_id_or_pop_state();
    return lookup_state(id);
}

int StateRegistry::get_bins_per_state() const {
    return state_packer.get_num_bins();
}

int StateRegistry::get_state_size_in_bytes() const {
    return get_bins_per_state() * sizeof(PackedStateBin);
}

void StateRegistry::print_statistics() const {
    cout << "Number of registered states: " << size() << endl;
    registered_states.print_statistics();
}

#undef BEGINF
#undef ENDF
