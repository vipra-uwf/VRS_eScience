import json
import codecs

# test_string = "\"\\\\[\\r\\n\\\\begin{equation*}\\r\\nm_{i} \\\\frac{d v_{i}}{d t}=m_{i} \\\\frac{d^{2} x_{i}}{d t^{2}}=f_{i}^{\\\\text {prop }}(t)+f_{i}^{\\\\text {rep }}(t) \\\\tag{1}\\r\\n\\\\end{equation*}\\r\\n\\\\]\\n\\n\\\\[\\r\\n\\\\begin{equation*}\\r\\nf_{i}^{\\\\text {prop }}=\\\\frac{\\\\left(v_{0 i}-v_{i}\\\\right)}{\\\\tau} \\\\times m_{i} \\\\tag{2}\\r\\n\\\\end{equation*}\\r\\n\\\\]\\n\\n\\\\[\\r\\n\\\\begin{equation*}\\r\\n\\\\alpha=1-\\\\frac{\\\\lambda}{d_{i}} \\\\tag{3}\\r\\n\\\\end{equation*}\\r\\n\\\\]\\n\\n\\\\[\\r\\n\\\\begin{equation*}\\r\\nV_{i j}^{L J}=\\\\epsilon \\\\frac{\\\\sigma}{r_{i j}^{12}} \\\\tag{4}\\r\\n\\\\end{equation*}\\r\\n\\\\]\\n\\n\\\\[\\r\\n\\\\begin{equation*}\\r\\nf_{i}^{r e p}=\\\\frac{(\\\\beta-1) v_{0 i}}{\\\\tau} \\\\times m_{i} \\\\tag{5}\\r\\n\\\\end{equation*}\\r\\n\\\\]\\n\\n\\\\[\\r\\n\\\\begin{equation*}\\r\\nm_{i} \\\\frac{d^{2} x_{i}}{d t^{2}}=\\\\frac{\\\\left(\\\\beta v_{0 i}-v_{i}\\\\right)}{\\\\tau} \\\\times m_{i} \\\\tag{6}\\r\\n\\\\end{equation*}\\r\\n\\\\]\\n\\n\\\\[\\r\\n\\\\begin{equation*}\\r\\n\\\\beta=c-e^{-a\\\\left(d_{i}-b\\\\right)} \\\\tag{7}\\r\\n\\\\end{equation*}\\r\\n\\\\]\\n\\n\\\\[\\r\\n\\\\begin{gather*}\\r\\n\\\\rho_{t}=\\\\frac{N_{t}}{A}  \\\\tag{8}\\\\\\\\\\r\\n\\\\overline{v_{t}}=\\\\frac{N_{t}}{\\\\sum_{i=1}^{N_{t}} \\\\frac{1}{N_{t}}} \\\\tag{9}\\r\\n\\\\end{gather*}\\r\\n\\\\]\\n\\n\\\\[\\r\\n\\\\begin{equation*}\\r\\nv_{F, h i}(D)=v_{F, h f} \\\\times\\\\left(1-e^{-\\\\lambda \\\\times\\\\left(\\\\frac{1}{D}-\\\\frac{1}{D_{\\\\max }}\\\\right)}\\\\right) \\\\tag{10}\\r\\n\\\\end{equation*}\\r\\n\\\\]\""
test_string = '```json{\n  \"utility_functions\": [\n    {\n      \"name\": \"norm\",\n      \"signature\": \"VIPRA::f_pnt norm(VIPRA::f3d const& vec)\",\n      \"body\": \"return std::sqrt(vec.x * vec.x + vec.y * vec.y + vec.z * vec.z);\\\\\"\n    },\n    {\n      \"name\": \"normalize\",\n      \"signature\": \"VIPRA::f3d normalize(VIPRA::f3d const& vec)\",\n      \"body\": \"VIPRA::f_pnt magnitude = norm(vec);\\\\\\nif (magnitude < 1e-10) {\\\\\\n  return VIPRA::f3d{0.0, 0.0, 0.0};\\\\\\n}\\\\\\nreturn VIPRA::f3d{vec.x / magnitude, vec.y / magnitude, vec.z / magnitude};\\\\\"\n    },\n    {\n      \"name\": \"distance_between\",\n      \"signature\": \"VIPRA::f_pnt distance_between(VIPRA::f3d const& a, VIPRA::f3d const& b)\",\n      \"body\": \"VIPRA::f_pnt dx = b.x - a.x;\\\\\\nVIPRA::f_pnt dy = b.y - a.y;\\\\\\nVIPRA::f_pnt dz = b.z - a.z;\\\\\\nreturn std::sqrt(dx*dx + dy*dy + dz*dz);\\\\\"\n    },\n    {\n      \"name\": \"direction_to_goal\",\n      \"signature\": \"VIPRA::f3d direction_to_goal(VIPRA::f3d const& position, VIPRA::f3d const& goal)\",\n      \"body\": \"VIPRA::f3d dir;\\\\\\ndir.x = goal.x - position.x;\\\\\\ndir.y = goal.y - position.y;\\\\\\ndir.z = goal.z - position.z;\\\\\\nreturn normalize(dir);\\\\\"\n    }\n  ],\n  \"primary_functions\": [\n    {\n      \"name\": \"propulsion\",\n      \"signature\": \"VIPRA::f3d propulsion(VIPRA::Modules::Pedestrians const& pedset, VIPRA::Modules::Map const& map, VIPRA::Modules::Goals const& goals, VIPRA::State& state, VIPRA::delta_t deltaT, VIPRA::timestep timestep, VIPRA::idx pedIdx)\",\n      \"body\": \"VIPRA::f3d current_velocity = pedset.ped_velocity(pedIdx);\\\\\\nVIPRA::f3d current_position = pedset.ped_coords(pedIdx);\\\\\\nVIPRA::f3d goal = goals.current_goal(pedIdx);\\\\\\nVIPRA::f3d desired_direction = direction_to_goal(current_position, goal);\\\\\\nVIPRA::f_pnt m_i = 80.0;\\\\\\nVIPRA::f_pnt v_0i = 1.34;\\\\\\nVIPRA::f_pnt tau = 0.5;\\\\\\nVIPRA::f3d desired_velocity;\\\\\\ndesired_velocity.x = v_0i * desired_direction.x;\\\\\\ndesired_velocity.y = v_0i * desired_direction.y;\\\\\\ndesired_velocity.z = v_0i * desired_direction.z;\\\\\\nVIPRA::f3d velocity_diff;\\\\\\nvelocity_diff.x = desired_velocity.x - current_velocity.x;\\\\\\nvelocity_diff.y = desired_velocity.y - current_velocity.y;\\\\\\nvelocity_diff.z = desired_velocity.z - current_velocity.z;\\\\\\nVIPRA::f_pnt scalar = (m_i / tau);\\\\\\nVIPRA::f3d f_prop;\\\\\\nf_prop.x = scalar * velocity_diff.x;\\\\\\nf_prop.y = scalar * velocity_diff.y;\\\\\\nf_prop.z = scalar * velocity_diff.z;\\\\\\nreturn f_prop;\\\\\"\n    },\n    {\n      \"name\": \"repulsion\",\n      \"signature\": \"VIPRA::f3d repulsion(VIPRA::Modules::Pedestrians const& pedset, VIPRA::Modules::Map const& map, VIPRA::Modules::Goals const& goals, VIPRA::State& state, VIPRA::delta_t deltaT, VIPRA::timestep timestep, VIPRA::idx pedIdx)\",\n      \"body\": \"VIPRA::f3d current_position = pedset.ped_coords(pedIdx);\\\\\\nVIPRA::f3d goal = goals.current_goal(pedIdx);\\\\\\nVIPRA::f_pnt d_i = distance_between(current_position, goal);\\\\\\nVIPRA::f_pnt a = 0.1;\\\\\\nVIPRA::f_pnt b = 1.0;\\\\\\nVIPRA::f_pnt c = 1.0;\\\\\\nVIPRA::f_pnt beta = c - std::exp(-a * (d_i - b));\\\\\\nVIPRA::f_pnt m_i = 80.0;\\\\\\nVIPRA::f_pnt v_0i = 1.34;\\\\\\nVIPRA::f_pnt tau = 0.5;\\\\\\nVIPRA::f_pnt scalar = ((beta - 1.0) * v_0i / tau) * m_i;\\\\\\nVIPRA::f3d desired_direction = direction_to_goal(current_position, goal);\\\\\\nVIPRA::f3d f_rep;\\\\\\nf_rep.x = scalar * desired_direction.x;\\\\\\nf_rep.y = scalar * desired_direction.y;\\\\\\nf_rep.z = scalar * desired_direction.z;\\\\\\nreturn f_rep;\\\\\"\n    },\n    {\n      \"name\": \"force_field\",\n      \"signature\": \"VIPRA::f3d force_field(VIPRA::Modules::Pedestrians const& pedset, VIPRA::Modules::Map const& map, VIPRA::Modules::Goals const& goals, VIPRA::State& state, VIPRA::delta_t deltaT, VIPRA::timestep timestep, VIPRA::idx pedIdx, VIPRA::f3d propulsion, VIPRA::f3d repulsion)\",\n      \"body\": \"VIPRA::f3d total_force;\\\\\\ntotal_force.x = propulsion.x + repulsion.x;\\\\\\ntotal_force.y = propulsion.y + repulsion.y;\\\\\\ntotal_force.z = propulsion.z + repulsion.z;\\\\\\nreturn total_force;\\\\\"\n    },\n    {\n      \"name\": \"update_ped\",\n      \"signature\": \"void update_ped(VIPRA::Modules::Pedestrians const& pedset, VIPRA::Modules::Map const& map, VIPRA::Modules::Goals const& goals, VIPRA::State& state, VIPRA::delta_t deltaT, VIPRA::timestep timestep, VIPRA::idx pedIdx, VIPRA::f3d force)\",\n      \"body\": \"VIPRA::f_pnt m_i = 80.0;\\\\\\nVIPRA::f3d acceleration;\\\\\\nacceleration.x = force.x / m_i;\\\\\\nacceleration.y = force.y / m_i;\\\\\\nacceleration.z = force.z / m_i;\\\\\\nVIPRA::f3d current_velocity = state.velocities[pedIdx];\\\\\\nVIPRA::f3d new_velocity;\\\\\\nnew_velocity.x = current_velocity.x + acceleration.x * deltaT;\\\\\\nnew_velocity.y = current_velocity.y + acceleration.y * deltaT;\\\\\\nnew_velocity.z = current_velocity.z + acceleration.z * deltaT;\\\\\\nstate.velocities[pedIdx] = new_velocity;\\\\\\nVIPRA::f3d current_position = state.positions[pedIdx];\\\\\\nVIPRA::f3d new_position;\\\\\\nnew_position.x = current_position.x + new_velocity.x * deltaT;\\\\\\nnew_position.y = current_position.y + new_velocity.y * deltaT;\\\\\\nnew_position.z = current_position.z + new_velocity.z * deltaT;\\\\\\nstate.positions[pedIdx] = new_position;\\\\\"\n    }\n  ],\n  \"notes\": \"The provided LaTeX equations define a pedestrian dynamics model. Equations (2) and (5-7) were used to construct the propulsion and repulsion forces. Equation (1) shows the total force is the sum of propulsion and repulsion, implemented in force_field. Equation (4) defines a Lennard-Jones potential V_ij^LJ but does not appear directly in the force equations and was not used. Equations (3), (8), (9), and (10) define auxiliary quantities (alpha, density, average velocity, and a free-flow velocity function) that are not directly part of the force computation and were not implemented. Several parameters are assumed as fixed values: m_i (mass) = 80.0 kg, v_0i (desired speed) = 1.34 m/s, tau (relaxation time) = 0.5 s, and for the beta calculation: a = 0.1, b = 1.0 m, c = 1.0. These parameters should ideally be configurable or computed based on pedestrian properties. The repulsion force magnitude depends on the distance to goal d_i via the beta function (equation 7). The equations do not specify interaction forces between pedestrians or with obstacles; only self-propulsion and a distance-dependent modulation are included.\"\n}\n```'

# {
#   \"utility_functions\": [
#     {
#       \"name\": \"compute_direction_to_goal\",
#       \"signature\": \"VIPRA::f3d compute_direction_to_goal(VIPRA::f3d const& position, VIPRA::f3d const& goal)\",
#       \"body\": \"VIPRA::f3d direction; \\\\\\ndirection.x = goal.x - position.x; \\\\\\ndirection.y = goal.y - position.y; \\\\\\ndirection.z = goal.z - position.z; \\\\\\nVIPRA::f_pnt mag = std::sqrt(direction.x * direction.x + direction.y * direction.y + direction.z * direction.z); \\\\\\nif (mag > 1e-9) { \\\\\\n  direction.x /= mag; \\\\\\n  direction.y /= mag; \\\\\\n  direction.z /= mag; \\\\\\n} else { \\\\\\n  direction.x = 0.0; \\\\\\n  direction.y = 0.0; \\\\\\n  direction.z = 0.0; \\\\\\n} \\\\\\nreturn direction;\"
#     },
#     {
#       \"name\": \"compute_beta\",
#       \"signature\": \"VIPRA::f_pnt compute_beta(VIPRA::f_pnt d_i, VIPRA::f_pnt a, VIPRA::f_pnt b, VIPRA::f_pnt c)\",
#       \"body\": \"return c - std::exp(-a * (d_i - b));\"
#     }
#   ],
#   \"primary_functions\": [
#     {
#       \"name\": \"propulsion\",
#       \"signature\": \"VIPRA::f3d propulsion(VIPRA::Modules::Pedestrians const& pedset, VIPRA::Modules::Map const& map, VIPRA::Modules::Goals const& goals, VIPRA::State& state, VIPRA::delta_t deltaT, VIPRA::timestep timestep, VIPRA::idx pedIdx)\",
#       \"body\": \"/* Parameters: v_0i (desired speed), tau (relaxation time), m_i (mass) */ \\\\\\nVIPRA::f_pnt v_0i = 1.34; \\\\\\nVIPRA::f_pnt tau = 0.5; \\\\\\nVIPRA::f_pnt m_i = 80.0; \\\\\\n \\\\\\nVIPRA::f3d position = state.positions[pedIdx]; \\\\\\nVIPRA::f3d velocity = state.velocities[pedIdx]; \\\\\\nVIPRA::f3d goal = goals.current_goal(pedIdx); \\\\\\n \\\\\\nVIPRA::f3d direction = compute_direction_to_goal(position, goal); \\\\\\n \\\\\\nVIPRA::f3d desired_velocity; \\\\\\ndesired_velocity.x = v_0i * direction.x; \\\\\\ndesired_velocity.y = v_0i * direction.y; \\\\\\ndesired_velocity.z = v_0i * direction.z; \\\\\\n \\\\\\nVIPRA::f3d force; \\\\\\nforce.x = ((desired_velocity.x - velocity.x) / tau) * m_i; \\\\\\nforce.y = ((desired_velocity.y - velocity.y) / tau) * m_i; \\\\\\nforce.z = ((desired_velocity.z - velocity.z) / tau) * m_i; \\\\\\n \\\\\\nreturn force;\"
#     },
#     {
#       \"name\": \"repulsion\",
#       \"signature\": \"VIPRA::f3d repulsion(VIPRA::Modules::Pedestrians const& pedset, VIPRA::Modules::Map const& map, VIPRA::Modules::Goals const& goals, VIPRA::State& state, VIPRA::delta_t deltaT, VIPRA::timestep timestep, VIPRA::idx pedIdx)\",
#       \"body\": \"/* Parameters: v_0i (desired speed), tau, m_i (mass), a, b, c for beta, d_i (distance to goal) */ \\\\\\nVIPRA::f_pnt v_0i = 1.34; \\\\\\nVIPRA::f_pnt tau = 0.5; \\\\\\nVIPRA::f_pnt m_i = 80.0; \\\\\\nVIPRA::f_pnt a = 1.0; \\\\\\nVIPRA::f_pnt b = 1.0; \\\\\\nVIPRA::f_pnt c = 1.5; \\\\\\n \\\\\\nVIPRA::f3d position = state.positions[pedIdx]; \\\\\\nVIPRA::f3d goal = goals.current_goal(pedIdx); \\\\\\n \\\\\\nVIPRA::f_pnt dx = goal.x - position.x; \\\\\\nVIPRA::f_pnt dy = goal.y - position.y; \\\\\\nVIPRA::f_pnt dz = goal.z - position.z; \\\\\\nVIPRA::f_pnt d_i = std::sqrt(dx*dx + dy*dy + dz*dz); \\\\\\n \\\\\\nVIPRA::f_pnt beta = compute_beta(d_i, a, b, c); \\\\\\n \\\\\\nVIPRA::f3d direction = compute_direction_to_goal(position, goal); \\\\\\n \\\\\\nVIPRA::f_pnt force_magnitude = ((beta - 1.0) * v_0i / tau) * m_i; \\\\\\n \\\\\\nVIPRA::f3d force; \\\\\\nforce.x = force_magnitude * direction.x; \\\\\\nforce.y = force_magnitude * direction.y; \\\\\\nforce.z = force_magnitude * direction.z; \\\\\\n \\\\\\nreturn force;\"
#     },
#     {
#       \"name\": \"force_field\",
#       \"signature\": \"VIPRA::f3d force_field(VIPRA::Modules::Pedestrians const& pedset, VIPRA::Modules::Map const& map, VIPRA::Modules::Goals const& goals, VIPRA::State& state, VIPRA::delta_t deltaT, VIPRA::timestep timestep, VIPRA::idx pedIdx, VIPRA::f3d propulsion, VIPRA::f3d repulsion)\",
#       \"body\": \"VIPRA::f3d total_force; \\\\\\ntotal_force.x = propulsion.x + repulsion.x; \\\\\\ntotal_force.y = propulsion.y + repulsion.y; \\\\\\ntotal_force.z = propulsion.z + repulsion.z; \\\\\\nreturn total_force;\"
#     },
#     {
#       \"name\": \"update_ped\",
#       \"signature\": \"void update_ped(VIPRA::Modules::Pedestrians const& pedset, VIPRA::Modules::Map const& map, VIPRA::Modules::Goals const& goals, VIPRA::State& state, VIPRA::delta_t deltaT, VIPRA::timestep timestep, VIPRA::idx pedIdx, VIPRA::f3d force)\",
#       \"body\": \"VIPRA::f_pnt m_i = 80.0; \\\\\\n \\\\\\nVIPRA::f3d acceleration; \\\\\\nacceleration.x = force.x / m_i; \\\\\\nacceleration.y = force.y / m_i; \\\\\\nacceleration.z = force.z / m_i; \\\\\\n \\\\\\nstate.velocities[pedIdx].x += acceleration.x * deltaT; \\\\\\nstate.velocities[pedIdx].y += acceleration.y * deltaT; \\\\\\nstate.velocities[pedIdx].z += acceleration.z * deltaT; \\\\\\n \\\\\\nstate.positions[pedIdx].x += state.velocities[pedIdx].x * deltaT; \\\\\\nstate.positions[pedIdx].y += state.velocities[pedIdx].y * deltaT; \\\\\\nstate.positions[pedIdx].z += state.velocities[pedIdx].z * deltaT;\"
#     }
#   ],
#   \"notes\": \"Parameters requiring specification:\\n\\n1. v_0i (desired speed): Fixed value, set to 1.34 m/s in implementation\\n2. tau (relaxation time): Fixed value, set to 0.5 s in implementation\\n3. m_i (pedestrian mass): Fixed value, set to 80.0 kg in implementation\\n4. a, b, c (beta function parameters): Fixed values for equation 7, set to a=1.0, b=1.0, c=1.5 in implementation\\n5. d_i (distance to goal): Computed from pedestrian position and goal position\\n\\nEquations 3, 4, 8, 9, and 10 were not incorporated as they appear to define auxiliary variables (alpha, Lennard-Jones potential, density, average velocity, and a free-flow velocity function) that are not directly used in the force calculations defined by equations 1, 2, 5, and 6. If these are needed for a more complete model, additional context about their role would be required.\\n\\nThe repulsion force (equation 5) uses the beta parameter from equation 7, which depends on distance to goal d_i. The propulsion force (equation 2) drives the pedestrian toward their desired velocity in the direction of their goal.\"
# }

# Get code block. Copy-paste?
print("Enter string obtained from code generation results (including the quotes):")
code_string = input()
# code_string = test_string


# trim json from the front and the triple ` from the front and end.
# code_string = code_string.substr(code_string.find('{'), code_string.rfind('}') - code_string.find('{') + 1)
# code_string = code_string[code_string.find('{'):code_string.rfind('}') + 1]


def extract_json_text(raw_text: str) -> str:
    raw_text = raw_text.strip()

    # Remove a surrounding quoted string if the user pasted the whole thing with leading/trailing quotes.
    if len(raw_text) >= 2 and ((raw_text[0] == raw_text[-1] == '"') or (raw_text[0] == raw_text[-1] == "'")):
        raw_text = raw_text[1:-1].strip()

    # If the string contains escape sequences (e.g., \n, \", \\) from a Python literal,
    # decode them to their actual characters.
    if '\\' in raw_text:
        raw_text = codecs.decode(raw_text, 'unicode_escape')

    # Extract the JSON object by locating the first '{' and last '}'.
    # This naturally ignores code fences and any other surrounding text.
    start_index = raw_text.find('{')
    end_index = raw_text.rfind('}')
    if start_index == -1 or end_index == -1 or end_index < start_index:
        raise ValueError('Could not find a JSON object in the pasted input.')

    return raw_text[start_index:end_index + 1]

code_string = extract_json_text(code_string)

print("\n\n" + code_string)

# Read JSON object.
json_object = json.loads(code_string)


# Fill Template.
code_template = ''

# headers
code_template += '#pragma once\n#include \"vipra/geometry/f3d.hpp\"\n#include \"vipra/modules/goals.hpp\"\n#include \"vipra/modules/pedestrians.hpp\"\n#include \"vipra/modules/model.hpp\"\n#include \"vipra/types/float.hpp\"\n#include \"vipra/types/idx.hpp\"\n#include \"vipra/macros/model.hpp\"\n\n'

# forcefield params
code_template += '#define FORCE_FIELD_PARAMS \n\n'

# repulsion function
code_template += '#define REPULSION_FORCE \\\n' + json_object["primary_functions"][0]["signature"] + '{ \\\n' + json_object["primary_functions"][0]["body"]
code_template += '\n}\\\n' if code_template[-1:] == '\\' else '\\\n}\\\n'
code_template += '\n\n'

# propulsion function
code_template += '#define PROPULSION_FORCE \\\n' + json_object["primary_functions"][1]["signature"] + '{ \\\n' + json_object["primary_functions"][1]["body"]
code_template += '\n}\\\n' if code_template[-1:] == '\\' else '\\\n}\\\n'
code_template += '\n\n'

# force field function
code_template += '#define FORCE_FIELD \\\n' + json_object["primary_functions"][2]["signature"] + '{ \\\n' + json_object["primary_functions"][2]["body"]
code_template += '\n}\\\n' if code_template[-1:] == '\\' else '\\\n}\\\n'
code_template += '\n\n'

# update state function
code_template += '#define UPDATE_STATE \\\n' + json_object["primary_functions"][3]["signature"] + '{ \\\n' + json_object["primary_functions"][3]["body"]
code_template += '\n}\\\n' if code_template[-1:] == '\\' else '\\\n}\\\n'
code_template += '\n\n'

# additional functions
code_template += '#define ADDITIONAL_SOCIAL_FORCE_FUNCTIONS \\\n'
for func in json_object["utility_functions"]:
    code_template += func["signature"] + '{\\\n' + func["body"]
    code_template += '\n}\\\n' if code_template[-1:] == '\\' else '\\\n}\\\n'

code_template += '\n\n'
code_template += '/* Notes:\n' + json_object["notes"] + '*/\n'

# Create File.
file = open("./SocialForce/force_field.hpp", "w")

# Write to file.
file.write(code_template)
file.close()
