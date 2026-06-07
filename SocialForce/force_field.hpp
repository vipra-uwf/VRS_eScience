#pragma once
#include "vipra/geometry/f3d.hpp"
#include "vipra/modules/goals.hpp"
#include "vipra/modules/pedestrians.hpp"
#include "vipra/modules/model.hpp"
#include "vipra/types/float.hpp"
#include "vipra/types/idx.hpp"
#include "vipra/macros/model.hpp"

#define FORCE_FIELD_PARAMS

#define REPULSION_FORCE \
VIPRA::f3d propulsion(VIPRA::Modules::Pedestrians const& pedset, VIPRA::Modules::Map const& map, VIPRA::Modules::Goals const& goals, VIPRA::State& state, VIPRA::delta_t deltaT, VIPRA::timestep timestep, VIPRA::idx pedIdx) { \

}\


#define PROPULSION_FORCE \
VIPRA::f3d repulsion(VIPRA::Modules::Pedestrians const& pedset, VIPRA::Modules::Map const& map, VIPRA::Modules::Goals const& goals, VIPRA::State& state, VIPRA::delta_t deltaT, VIPRA::timestep timestep, VIPRA::idx pedIdx) { \

}\


#define FORCE_FIELD \
VIPRA::f3d force_field(VIPRA::Modules::Pedestrians const& pedset, VIPRA::Modules::Map const& map, VIPRA::Modules::Goals const& goals, VIPRA::State& state, VIPRA::delta_t deltaT, VIPRA::timestep timestep, VIPRA::idx pedIdx, VIPRA::f3d propulsion, VIPRA::f3d repulsion) { \

}\


#define UPDATE_PED \
void update_ped(VIPRA::Modules::Pedestrians const& pedset, VIPRA::Modules::Map const& map, VIPRA::Modules::Goals const& goals, VIPRA::State& state, VIPRA::delta_t deltaT, VIPRA::timestep timestep, VIPRA::idx pedIdx, VIPRA::f3d force) { \

}\


#define ADDITIONAL_SOCIAL_FORCE_FUNCTIONS \



