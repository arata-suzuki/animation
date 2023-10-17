#include "constraint.hpp"

using namespace cgp;

// Helper function to hash two integers into a single one for the map
static size_t hash_pair(int ku, int kv)
{
    return size_t(ku) + 1000 * size_t(kv);
}

void constraint_structure::add_fixed_position(int ku, int kv, vec3 position)
{
    fixed_sample.push_back(position_contraint(ku,kv, position));
}
void constraint_structure::remove_fixed_position(int ku, int kv)
{
    //fixed_sample.erase(hash_pair(ku, kv));
}