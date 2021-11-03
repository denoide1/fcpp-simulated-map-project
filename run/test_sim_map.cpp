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
}

//! @brief Main function.
MAIN() {

    node.storage(tags::node_size{}) = 10;
    node.storage(tags::node_color{}) = color(GREEN);
    node.storage(tags::node_shape{}) = shape::sphere;

    old(CALL, 0, [&](int a){  // used to set position of out of bound nodes at the start
        if (a == 0 && node.net.is_obstacle(node.position())) {
            auto p2 = node.net.closest_space(node.position());
            int deltaX, deltaY, size = node.storage(tags::node_size{});
            if((p2 - node.position())[0] > 0) deltaX = +size; else deltaX = -size;
            if((p2 - node.position())[1] > 0) deltaY = +size; else deltaY = -size;
            node.position() = make_vec(p2[0] + deltaX, p2[1] + deltaY);
        }
        return a+1;
    });
    //500 - node.position()[1] is needed to convert Y axis, renderer considers origin at (0.0) at bottom left of the viewport, bitmap considers origin (0,0) at top left of the viewport
    auto position = make_vec(node.position()[0], height - node.position()[1]);
    auto closest = make_vec(node.net.closest_obstacle(position)[0], height - node.net.closest_obstacle(position)[1]);
    auto dist1 = distance(closest, node.position());

    node.storage(tags::nearest_obstacle{}) = closest;
    node.storage(tags::distance_from_obstacle{}) = dist1;

    if(dist1 <= 20) {
        node.velocity() = make_vec(0,0);
        int speed = node.storage(tags::speed{});
        auto diff_vec = closest - node.position();
        node.storage(tags::obstacle_delta_x{}) = diff_vec[0];
        node.storage(tags::obstacle_delta_y{}) = diff_vec[1];
        if(diff_vec[0] > 0) node.velocity() = make_vec(-speed,node.velocity()[1]); else node.velocity() = make_vec(speed,node.velocity()[1]);
        if(diff_vec[1] > 0) node.velocity() = make_vec(node.velocity()[0],-speed); else node.velocity() = make_vec(node.velocity()[0],speed);
    }
    else {
        rectangle_walk(CALL, make_vec(0,0), make_vec(width,height), node.storage(tags::speed{}), 0);
    }

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
constexpr int node_num = 10;
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
using rectangle_d = distribution::rect_n<1, 0, 0, 500, 500>;
//! @brief The distribution of node speeds (all equal to a fixed value).
using speed_d = distribution::constant_i<double, speed>;
//! @brief The contents of the node storage as tags and associated types.
using store_t = tuple_store<
    nearest_obstacle,           vec<2>,
    distance_from_obstacle,     real_t,
    obstacle_delta_x,           real_t,
    obstacle_delta_y,           real_t,
    speed,                      double,
    node_color,                 color,
    node_size,                  double,
    node_shape,                 shape
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
    area<0,0,500,500,1>
);

} // namespace option

} // namespace fcpp

//! @brief The main function.
int main() {
    using namespace fcpp;
    //! @brief The network object type (interactive simulator with given options).
    using net_t = component::interactive_simulator<option::list>::net;
    //! @brief The initialisation values (simulation name).
    auto init_v = common::make_tagged_tuple<option::name, option::texture, option::obstacles, option::speed>("Simulated map test", "building.png", "building.png", 5);
    //! @brief Construct the network object.
    net_t network{init_v};
    //! @brief Run the simulation until exit.
    network.run();
    return 0;
}
