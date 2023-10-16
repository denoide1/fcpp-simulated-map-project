// Copyright © 2021 Gianmarco Rampulla. All Rights Reserved.

/**
 * @file test_sim_map.cpp
 * @brief Quick experiment for simulated map component.
 */

// [INTRODUCTION]
//! Importing the FCPP library.
#include "lib/fcpp.hpp"
/**
 * @brief Namespace containing all the objects in the FCPP library.
 */
namespace fcpp {

//! @brief Dummy ordering between positions (allows positions to be used as secondary keys in ordered tuples).
template <size_t n>
bool operator<(vec<n> const& a, vec<n> const& b) {
    for(int i = 0; i < n; i++)
        if (a[i] >= b[i]) return  false;
    return true;
}

std::ostream& operator<<(std::ostream& o, map_navigator const&) {
    return o;
}

//! @brief Dimensionality of the space.
constexpr size_t dim = 2;
//! @brief Side of the deployment area.
constexpr size_t width = 500;
//! @brief Height of the deployment area.
constexpr size_t height = 500;

//! @brief Namespace containing the libraries of coordination routines.
namespace coordination {

//! @brief Tags used in the node storage.
namespace tags {
    //! @brief Color of the current node.
    struct node_color {};
    //! @brief Size of the current node.
    struct node_size {};
    //! @brief Shape of the current node.
    struct node_shape {};
    //! @brief Speed of the current node
    struct speed {};
    //! @brief Coordinates of nearest obstacle
    struct nearest_obstacle {};
    //! @brief Distance from nearest obstacle
    struct distance_from_obstacle {};
    //! @brief Delta X from nearest obstacle
    struct obstacle_delta_x {};
    //! @brief Delta Y from nearest obstacle
    struct obstacle_delta_y {};
    //! @brief Distance from closest neighbour
    struct distance_min_nbr {};
    
    struct current_room {};

    struct dest_room {};
    
    struct current_waypoint {};

    struct current_room_waypoint {};

    struct dest_room_waypoint {};

    struct distances_room_waypoints {};
}


<<<<<<< HEAD
//! @brief Generates a random target in a rectangle, avoiding obstacles.
template <typename node_t, size_t n>
inline vec<n> random_valid_target(node_t& node, trace_t call_point, vec<n> const& low, vec<n> const& hi) {
    return node.net.closest_space(random_rectangle_target(node, call_point, low, hi));
}


//! @brief Checks that the result of path_to makes sense as a target.
void check_correctness(device_t id, bool still, vec<2> t, real_t d) {
    if (isnan(d))
        std::cerr << id << " ERRORE: distanza not-a-number " << d << std::endl;
    else if (isinf(d) and d > 0 and still); // fine
    else if (d < 0 or d > width+height)
        std::cerr << id << " ERRORE: distanza fuori dai limiti: " << d << std::endl;
    if (isnan(t[0]) or isnan(t[1]))
        std::cerr << id << " ERRORE: target not-a-number " << t << std::endl;
    if (t[0] < 0 or t[1] < 0 or t[0] > width or t[1] > height)
        std::cerr << id << " ERRORE: target fuori dalla mappa " << t << std::endl;
}


//! @brief Walks randomly in a rectangle at a fixed speed.
template <typename node_t, size_t n>
inline vec<n> rectangle_walk_avoiding_obstacles(node_t& node, trace_t call_point, vec<n> const& low, vec<n> const& hi, real_t max_v, real_t period) {
    vec<n> target = random_valid_target(node, call_point, low, hi);
    return old(node, call_point, target, [&](vec<n> t){
        std::pair<vec<n>, real_t> waypoint_dist = node.net.path_to(node.net.closest_space(node.position()), t, node.uid == 0);
        node.storage(tags::current_target{}) = t;
        node.storage(tags::current_room_waypoint{}) = waypoint_dist.first;
        node.storage(tags::dest_room_waypoint{}) = waypoint_dist.second;
        check_correctness(node.uid, waypoint_dist.first == node.position(), waypoint_dist.first, waypoint_dist.second);
        follow_target(node, call_point, waypoint_dist.first, max_v, period);
        return waypoint_dist.second > max_v * period ? t : target;
    });
}
template <size_t n> using rectangle_walk_avoiding_obstacles_t = common::export_list<vec<n>>;

=======
>>>>>>> parent of 839bbf4 (improving test simulation)

//! @brief Repulsive force from obstacles.
FUN void repel_obstacles(ARGS, real_t strength) { CODE
    if (node.net.is_obstacle(node.position())) {
        node.propulsion() = unit(node.net.closest_space(node.position()) - node.position()) * strength;
    } else {
        vec<2> t = node.position() - node.net.closest_obstacle(node.position());
        real_t d = norm(t);
        if (d < node.storage(tags::node_size{}))
            node.propulsion() = unit(t) * strength * (node.storage(tags::node_size{}) - d) / node.storage(tags::node_size{});
        else
            node.propulsion() = make_vec(0,0);
    }
}


//! @brief Main function.
MAIN() {
    node.storage(tags::node_size{}) = 10;
    node.storage(tags::node_color{}) = color(GREEN);
    node.storage(tags::node_shape{}) = shape::sphere;
    
    // used to set position of out of bound nodes at the start
    if (coordination::counter(CALL) == 1) {
        if (node.net.is_obstacle(node.position())) {
            auto p2 = node.net.closest_space(node.position());
            int deltaX, deltaY, size = node.storage(tags::node_size{});
            if((p2 - node.position())[0] > 0) deltaX = +size; else deltaX = -size;
            if((p2 - node.position())[1] > 0) deltaY = +size; else deltaY = -size;
            node.position() = make_vec(p2[0] + deltaX, p2[1] + deltaY);
        }
    }

    /*
	node.storage(tags::current_room{}) = node.net.get_current_room(node.position());
    node.storage(tags::dest_room{}) = node.net.get_current_room(make_vec(300, 300));
    node.storage(tags::current_room_waypoint{}) = node.net.get_waypoints_room(node.position());
    node.storage(tags::dest_room_waypoint{}) = node.net.get_waypoints_room(make_vec(300, 300));
    node.storage(tags::distances_room_waypoints{}) = node.net.get_distance_from(node.position(),make_vec(300, 300));
     */

    vec<2> next_step = node.net.path_to(node.position(),make_vec(300, 300));


    /*
    if(std::abs(node.position()[0] - next_step[0]) > 0) {
        next_step[0] += 1;
    }
        next_step[1] += 1;
    next_step[1] += 1;
     */



    node.storage(tags::current_waypoint{}) = next_step;
    follow_target(CALL,next_step,20,1);
    
    
>>>>>>> parent of 839bbf4 (improving test simulation)
}
//! @brief Export types used by the main function (update it when expanding the program).
FUN_EXPORT main_t = common::export_list<double, int, rectangle_walk_t<2>>;

} // namespace coordination

// [SYSTEM SETUP]

//! @brief Namespace for component options.
namespace option {

//! @brief Import tags to be used for component options.
using namespace component::tags;
//! @brief Import tags used by aggregate functions.
using namespace coordination::tags;

//! @brief Number of people in the area.
constexpr int node_num = 5;
//! @brief Dimensionality of the space.
constexpr size_t dim = 2;

//! @brief Description of the round schedule.
using round_s = sequence::periodic<
    distribution::interval_n<times_t, 0, 1>,    // uniform time in the [0,1] interval for start
    distribution::weibull_n<times_t, 10, 1, 10> // weibull-distributed time for interval (10/10=1 mean, 1/10=0.1 deviation)
>;
//! @brief The sequence of network snapshots (one every simulated second).
using log_s = sequence::periodic_n<1, 0, 1>;
//! @brief The sequence of node generation events (node_num devices all generated at time 0).
using spawn_s = sequence::multiple_n<node_num, 0>;
//! @brief The distribution of initial node positions (random in a 500x500 square).
using rectangle_d = distribution::rect_n<1, 0, 0, 735, 418>;
//! @brief The distribution of node speeds (all equal to a fixed value).
using speed_d = distribution::constant_i<double, speed>;
//! @brief The contents of the node storage as tags and associated types.
using store_t = tuple_store<
    nearest_obstacle,           vec<2>,
    distance_from_obstacle,     real_t,
    obstacle_delta_x,           real_t,
    obstacle_delta_y,           real_t,
    distance_min_nbr,           real_t,
    speed,                      double,
    node_color,                 color,
    node_size,                  double,
    node_shape,                 shape,
    current_room,				std::vector<int>,
    dest_room,                  std::vector<int>,
    current_waypoint,			vec<2>,
    current_room_waypoint,      std::vector<std::array<size_t,2>>,
    dest_room_waypoint,         std::vector<std::array<size_t,2>>,
    distances_room_waypoints,  std::vector<real_t>
>;
//! @brief The tags and corresponding aggregators to be logged (change as needed).
using aggregator_t = aggregators<
    node_size,                  aggregator::mean<double>
>;

//! @brief The general simulation options.
DECLARE_OPTIONS(list,
    parallel<true>,      // multithreading enabled on node rounds
    synchronised<false>, // optimise for asynchronous networks
    program<coordination::main>,   // program to be run (refers to MAIN above)
    exports<coordination::main_t>, // export type list (types used in messages)
    retain<metric::retain<2,1>>,   // messages are kept for 2 seconds before expiring
    round_schedule<round_s>, // the sequence generator for round events on nodes
    log_schedule<log_s>,     // the sequence generator for log events on the network
    spawn_schedule<spawn_s>, // the sequence generator of node creation events on the network
    store_t,       // the contents of the node storage
    aggregator_t,  // the tags and corresponding aggregators to be logged
    init<
        x,     rectangle_d, // initialise position randomly in a rectangle for new nodes
        speed, speed_d
    >,
    dimension<dim>, // dimensionality of the space
    connector<connect::fixed<100, 1, dim>>, // connection allowed within a fixed comm range
    shape_tag<node_shape>, // the shape of a node is read from this tag in the store
    size_tag<node_size>,   // the size  of a node is read from this tag in the store
    color_tag<node_color>,  // the color of a node is read from this tag in the store
    area<0,0,1200,800,1>
);

} // namespace option

} // namespace fcpp

//! @brief The main function.
int main() {
    using namespace fcpp;
    //! @brief The network object type (interactive simulator with given options).
    using net_t = component::interactive_simulator<option::list>::net;

    map_navigator obj = map_navigator("oslo2.png");
     //! @brief The initialisation values (simulation name).
    auto init_v = common::make_tagged_tuple<option::name, option::texture, fcpp::component::tags::map_navigator_obj, option::speed>("Simulated map test", "./debugPointsOslo.png", obj, 1.5);
    //! @brief Construct the network object.
    net_t network{init_v};
    //! @brief Run the simulation until exit.
    network.run();
    return 0;
}
