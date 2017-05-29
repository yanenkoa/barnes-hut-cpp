#include <iostream>
#include <array>
#include <limits>
#include <memory>
#include <queue>
#include <random>
#include <SFML/Graphics.hpp>
#include "Eigen/Eigen"
#include <chrono>
#include <thread>
#include <fstream>

using namespace Eigen;

IOFormat PrettyFmt(StreamPrecision, DontAlignCols, ", ", ", ", "", "", "[", "]");

struct Body
{
    int id;
    double mass;
    Vector3d location;
    Vector3d velocity;
    Vector3d acceleration;
};

std::ostream &operator<<(std::ostream &os, const Body &body)
{
    os << "{ id = " << body.id
       << ", mass = " << body.mass
       << ", point = " << body.location.format(PrettyFmt)
       << ", velocity = " << body.velocity.format(PrettyFmt)
       << ", acceleration = " << body.acceleration.format(PrettyFmt)
       << " }";
    return os;
}

enum NodeType
{
    EMPTY_EXTERNAL, NONEMPTY_EXTERNAL, INTERNAL
};

class OcNode
{
private:
    Vector3d get_child_lower_vertice(std::size_t child_index)
    {
        Vector3d result = {0, 0, 0};
        if (child_index == 0 || child_index == 3 || child_index == 4 || child_index == 7) {
            result[0] = lower_vertice[0];
        } else {
            result[0] = (upper_vertice[0] + lower_vertice[0]) / 2;
        }
        if (child_index == 0 || child_index == 1 || child_index == 4 || child_index == 5) {
            result[1] = lower_vertice[1];
        } else {
            result[1] = (upper_vertice[1] + lower_vertice[1]) / 2;
        }
        if (child_index == 0 || child_index == 1 || child_index == 2 || child_index == 3) {
            result[2] = lower_vertice[2];
        } else {
            result[2] = (upper_vertice[2] + lower_vertice[2]) / 2;
        }
        return result;
    }

    Vector3d get_child_upper_vertice(std::size_t child_index)
    {
        Vector3d result = {0, 0, 0};
        if (child_index == 0 || child_index == 3 || child_index == 4 || child_index == 7) {
            result[0] = (upper_vertice[0] + lower_vertice[0]) / 2;
        } else {
            result[0] = upper_vertice[0];
        }
        if (child_index == 0 || child_index == 1 || child_index == 4 || child_index == 5) {
            result[1] = (upper_vertice[1] + lower_vertice[1]) / 2;
        } else {
            result[1] = upper_vertice[1];
        }
        if (child_index == 0 || child_index == 1 || child_index == 2 || child_index == 3) {
            result[2] = (upper_vertice[2] + lower_vertice[2]) / 2;
        } else {
            result[2] = upper_vertice[2];
        }
        return result;
    }

    std::size_t get_child_index(const Vector3d &point)
    {
        if (point[0] < centerpoint[0]) {
            if (point[1] < centerpoint[1]) {
                if (point[2] < centerpoint[2]) {
                    return 0;
                } else {
                    return 4;
                }
            } else {
                if (point[2] < centerpoint[2]) {
                    return 3;
                } else {
                    return 7;
                }
            }
        } else {
            if (point[1] < centerpoint[1]) {
                if (point[2] < centerpoint[2]) {
                    return 1;
                } else {
                    return 5;
                }
            } else {
                if (point[2] < centerpoint[2]) {
                    return 2;
                } else {
                    return 6;
                }
            }
        }
    }

    Vector3d get_force(const Body &body)
    {
        Vector3d diff_vector = center_of_mass.location - body.location;
        double dist = diff_vector.norm();
        if (type == NONEMPTY_EXTERNAL && center_of_mass.id == body.id) {
            return {0, 0, 0};
        }
        double force_value = G * center_of_mass.mass * body.mass / (dist * dist);
        Vector3d result = force_value * diff_vector.normalized();
        return result;
    }

    static constexpr double eps = std::numeric_limits<double>::epsilon();
    static constexpr double G = 6.67408e-11 * 5e9;
    static constexpr double theta = 0.0;

    NodeType type = EMPTY_EXTERNAL;
    Body center_of_mass;

    friend int main();

    friend void test_shit();

    friend void print_shit(const std::shared_ptr<OcNode> &);

public:
    std::array<std::shared_ptr<OcNode>, 8> children{
            nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr
    };
    const Vector3d lower_vertice, upper_vertice;
    const Vector3d centerpoint;

    OcNode(
            const Vector3d &lower_vertice_,
            const Vector3d &upper_vertice_
    )
            : lower_vertice(lower_vertice_), upper_vertice(upper_vertice_)
            , centerpoint((lower_vertice_ + upper_vertice_) / 2)
            , center_of_mass({-1, 0, centerpoint})
    {
    }

    void insert_point(const Body &body)
    {
        if (type == NONEMPTY_EXTERNAL) {
            type = INTERNAL;
            std::size_t i = get_child_index(center_of_mass.location);
            std::size_t curr_body_child_index = get_child_index(center_of_mass.location);
            children[curr_body_child_index].reset(new OcNode(
                    get_child_lower_vertice(curr_body_child_index),
                    get_child_upper_vertice(curr_body_child_index)
            ));
            children[curr_body_child_index]->insert_point(center_of_mass);
        }

        double mass_prev = center_of_mass.mass;
        Vector3d point_prev = center_of_mass.location;
        double mass_new = mass_prev + body.mass;
        Vector3d point_new = (mass_prev * point_prev + body.mass * body.location) / mass_new;
        center_of_mass = {body.id, mass_new, point_new};

        if (type == EMPTY_EXTERNAL) {
            type = NONEMPTY_EXTERNAL;
            return;
        }

        std::size_t child_index = get_child_index(body.location);
        if (children[child_index] == nullptr) {
            children[child_index].reset(new OcNode(
                    get_child_lower_vertice(child_index),
                    get_child_upper_vertice(child_index)
            ));
        }
        children[child_index]->insert_point(body);
    }

    Vector3d calculate_force(const Body &body)
    {
        switch (type) {
            case EMPTY_EXTERNAL: {
                return {0, 0, 0};
            }
            case NONEMPTY_EXTERNAL: {
                return get_force(body);
            }
            case INTERNAL: {
                double s = upper_vertice[0] - lower_vertice[0];
                double d = (center_of_mass.location - body.location).norm();
                if (s / d < theta) {
                    return get_force(body);
                } else {
                    Vector3d result;
                    for (auto child : children) {
                        if (child != nullptr) {
                            result += child->calculate_force(body);
                        }
                    }
                    return result;
                }
            }
            default: {
                exit(1488);
            }
        }
    }
};

class BarnesHut
{
private:
    std::vector<Body> &bodies;
    const double delta_t;
    const double max_time;

    Vector3d get_lower_vertice()
    {
        Vector3d result;
        for (auto body : bodies) {
            result[0] = result[0] < body.location[0] ? result[0] : body.location[0];
            result[1] = result[1] < body.location[1] ? result[1] : body.location[1];
            result[2] = result[2] < body.location[2] ? result[2] : body.location[2];
        }
        return result;
    }

    Vector3d get_upper_vertice()
    {
        Vector3d result;
        for (auto body : bodies) {
            result[0] = result[0] > body.location[0] ? result[0] : body.location[0];
            result[1] = result[1] > body.location[1] ? result[1] : body.location[1];
            result[2] = result[2] > body.location[2] ? result[2] : body.location[2];
        }
        return result;
    }

    void simulate_one_step()
    {

    }

    friend void test_shit_6();

public:
    BarnesHut(std::vector<Body> &bodies_, double delta_t_, double max_time_)
            : bodies(bodies_), delta_t(delta_t_), max_time(max_time_)
    {}

    void simulate(std::function<void(const std::vector<Body>&)> consume)
    {
        IOFormat MyFmt(StreamPrecision, DontAlignCols, ",", ", ", "", "", "[", "]");
        for (double curr_time = 0; curr_time < max_time; curr_time += delta_t) {
            auto lower = get_lower_vertice();
            auto upper = get_upper_vertice();
            auto diff = upper - lower;
            auto actual_lower = lower - 0.1 * diff;
            auto actual_upper = upper + 0.1 * diff;

            OcNode root(actual_lower, actual_upper);

            for (auto &body : bodies) {
                root.insert_point(body);
            }

            for (auto &body : bodies) {
                body.location += delta_t * body.velocity;
                body.velocity += delta_t * body.acceleration;
                Vector3d force = root.calculate_force(body);
                body.acceleration = force / body.mass;
            }

            consume(bodies);
        }
    }
};

void print_shit(const std::shared_ptr<OcNode> &node)
{
    std::cout << "\n";
    int eight_degree = 1;
    std::queue<std::shared_ptr<OcNode>> q;
    q.push(node);
    int j = 0;
    while (q.size() != 0) {
        std::shared_ptr<OcNode> curr_node = q.front();
        q.pop();
        if (curr_node == nullptr) {
            std::cout << 0;
        } else {
            switch (curr_node->type) {
                case EMPTY_EXTERNAL: {
                    std::cout << "E";
                    break;
                }
                case NONEMPTY_EXTERNAL: {
                    std::cout << "N";
                    break;
                }
                case INTERNAL: {
                    std::cout << "I";
                    break;
                }
            }
            for (auto child : curr_node->children) {
                q.push(child);
            }
        }
        ++j;
        if (j == eight_degree || q.size() == 0) {
            std::cout << "\n";
            for (int i = 0; i < j; ++i) {
                std::cout << i % 10;
            }
            std::cout << "\n";
            for (int i = 0; i < j; i += 10) {
                std::cout << i / 10 << "         ";
            }
            eight_degree *= 8;
            j = 0;
            std::cout << "\n===================================================\n";
        }
    }
}

void test_shit()
{
    std::cout << std::numeric_limits<double>::max() << " " << std::numeric_limits<double>::min() << "\n";

    std::shared_ptr<OcNode> node(new OcNode({0, 0, 0}, {4, 4, 4}));

    std::vector<Vector3d> norm_shifts{
            {0, 0, 0},
            {1, 0, 0},
            {1, 1, 0},
            {0, 1, 0},
            {0, 0, 1},
            {1, 0, 1},
            {1, 1, 1},
            {0, 1, 1},
    };

    double cross_coef = 2;

    for (int i = 0; i < norm_shifts.size(); ++i) {
        node->insert_point({i, 1, Vector3d({0.7, 0.7, 0.7}) + cross_coef * norm_shifts[i]});
    }
    node->insert_point({8, 1, {0.2, 0.2, 0.2}});

    std::cout << OcNode::G << "\n";

    IOFormat CommaInitFmt(StreamPrecision, DontAlignCols, ",", ", ", "", "", "[", "]");

    for (std::size_t i = 0; i < 8; ++i) {
        for (std::size_t j = 0; j < 8; ++j) {
            assert(node->children[i]->get_child_index(
                    (cross_coef * norm_shifts[j] + Vector3d(1, 1, 1)) / 2 + cross_coef * norm_shifts[i]
            ) == j);
        }

        std::cout << node->get_child_lower_vertice(i).format(CommaInitFmt) << " "
                  << node->get_child_upper_vertice(i).format(CommaInitFmt) << "\n";
    }

    print_shit(node);
}

void test_shit_2()
{
    std::vector<Body> bodies = {{1},
                                {1},
                                {2},
                                {3}};

    for (auto body : bodies) {
        std::cout << body << "\n";
    }

    for (auto &body : bodies) {
        body.mass += 1;
    }

    for (auto body : bodies) {
        std::cout << body << "\n";
    }
}

void test_shit_3()
{
    std::vector<Body> bodies = {
            {0, 5.972e24, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}},
            {1, 7.34767309e22, {3.844e8, 0, 0}, {0, 0, 0}, {0, 0, 0}}
    };

    std::shared_ptr<OcNode> root(new OcNode({-3.844e+07, 0, -6.95251e-311}, {4.2284e+08, 0, 7.64776e-310}));

    for (auto body : bodies) {
        root->insert_point(body);
    }

    print_shit(root);
}

void test_shit_4()
{
    std::vector<Body> bodies = {
            {0, 5.97200000e24, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}},
            {1, 7.34767309e22, {3.844e6, 0, 0}, {0, 0, 0}, {0, 0, 0}}
//            {1, 7.34767309e22, {3.844e8, 0, 0}, {0, 0, 0}, {0, 0, 0}}
    };

    BarnesHut bh(bodies, 1.0 / 60, 3);
    bh.simulate([](std::vector<Body> const &bodies) -> void
                {
                    for (Body body : bodies) std::cout << body << "\n";
                    std::cout << "iter_end\n";
                });
}

std::vector<Body> generate_bodies(std::size_t n_bodies)
{
    const double mass_lower = 1e30, mass_upper = 1e31;
    const double moon_coord = 3.844e7;
    const double location_abs_lower = moon_coord * 1000, location_abs_upper = moon_coord * 2000;
    const double velocity_abs_lower = 1e9, velocity_abs_upper = 1e11;

    std::uniform_real_distribution<double> mass_distr(mass_lower, mass_upper);
    std::uniform_real_distribution<double> location_abs_distr(location_abs_lower, location_abs_upper);
    std::uniform_real_distribution<double> velocity_abs_distr(velocity_abs_lower, velocity_abs_upper);

    long seed = std::chrono::system_clock::now().time_since_epoch().count();
//    long seed = 1488;
    std::default_random_engine generator(seed);

    std::vector<Body> bodies(n_bodies);
    for (int i = 0; i < n_bodies; ++i) {
        bodies[i].id = i;
        bodies[i].mass = mass_distr(generator);
        bodies[i].location = location_abs_distr(generator) * Vector3d::Random(3, 1);
        bodies[i].velocity = velocity_abs_distr(generator) * Vector3d::Random(3, 1);
        std::cout << bodies[i] << "\n";
    }

    return bodies;
}

void test_shit_5()
{
    using std::chrono::system_clock;
    using std::chrono::time_point;
    using std::chrono::milliseconds;
    using std::chrono::nanoseconds;
    using std::chrono::duration_cast;
    using std::chrono::duration;

    std::vector<Body> bodies = generate_bodies(1000);

//    const double moon_x_coord = 3.844e7;
//    const double moon_mass = 7.34767309e22;
    const double earth_mass = 5.9720e24;
//    std::vector<Body> bodies = {
//            {0, earth_mass, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}},
//            {1, moon_mass, {moon_x_coord, 0, 0}, {0, 1022000, 0}, {0, 0, 0}},
//            {2, moon_mass, {moon_x_coord / 2, 0, 0}, {0, -1022000, 0}, {0, 0, 0}},
//            {3, moon_mass, {-moon_x_coord * 3 / 4, 0, 0}, {0, -1000000, 0}, {0, 0, 0}},
//            {4, moon_mass, {-moon_x_coord / 2, moon_x_coord / 2, 0}, {0, 1000000, 0}, {0, 0, 0}},
//            {5, moon_mass, {-moon_x_coord / 2, -moon_x_coord / 2, 0}, {0, 700000, 0}, {0, 0, 0}},
//            {6, moon_mass, {moon_x_coord / 2, -moon_x_coord / 2, 0}, {0, -700000, 0}, {0, 0, 0}},
//    };

    Body central_body = {(int)bodies.size(), earth_mass * 1e8, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
    bodies.push_back(central_body);

    double max_coord = std::accumulate(
            bodies.begin(),
            bodies.end(),
            0.0,
            [&bodies](double acc, const Body &body) -> double
            {
                return std::max({acc, body.location[0], body.location[1], body.location[2]});
            }
    );

    const double delta_t = 1.0 / 30;
    const double max_time = 10;
    std::vector<std::vector<Body>> iterations;
    BarnesHut bh(bodies, delta_t, max_time);
    time_point<system_clock> before = system_clock::now();
    bh.simulate([&iterations](std::vector<Body> const &bodies) -> void
                {
                    iterations.push_back(std::vector<Body>());
                    for (Body body : bodies) {
                        iterations.back().push_back(Body(body));
                    }
                });
    time_point<system_clock> after = system_clock::now();
    double seconds = duration_cast<milliseconds>(after - before).count() / 1000;
    std::cout << seconds << " seconds\n";

    double min_mass = std::accumulate(
            bodies.begin(),
            bodies.end(),
            std::numeric_limits<double>::infinity(),
            [&bodies](double acc, const Body & body) -> double
            {
                return std::min(acc, body.mass);
            }
    );

    for (auto &iteration : iterations) {
        for (auto &body : iteration) {
            body.location *= 500 / max_coord * 0.5;
            body.location -= Vector3d(500, 500, 0);
        }
    }

    std::vector<sf::CircleShape> shapes;
    for (auto body : iterations[0]) {
        sf::CircleShape shape(3 * std::log(body.mass / min_mass + 10));
        shape.setFillColor(sf::Color::Green);
        shape.setOrigin(body.location[0], body.location[1]);
        shapes.push_back(shape);
    }

    sf::RenderWindow window(sf::VideoMode(1000, 1000), "Simulation");
    time_point<system_clock> prev_time = system_clock::now();
    int nanoseconds_in_frame = 1e9 * delta_t;
    int cur_i = 0;
    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed) {
                window.close();
            }
        }

        window.clear();

        auto curr_time = system_clock::now();
        if (duration_cast<nanoseconds>(curr_time - prev_time).count() >= nanoseconds_in_frame) {
            prev_time = curr_time;
            for (int i = 0; i < shapes.size(); ++i) {
                shapes[i].setOrigin(iterations[cur_i][i].location[0], iterations[cur_i][i].location[1]);
            }
            ++cur_i;
            if (cur_i >= iterations.size()) {
                window.close();
            }
        }

        for (auto shape : shapes) {
            window.draw(shape);
        }
        window.display();
    }
}

void test_shit_6()
{
    using std::chrono::system_clock;
    using std::chrono::time_point;
    using std::chrono::milliseconds;
    using std::chrono::nanoseconds;
    using std::chrono::duration_cast;
    using std::chrono::duration;

    int n_bodies = 1000;
    std::vector<Body> bodies = generate_bodies(n_bodies);

    const double earth_mass = 5.9720e24;

    Body central_body = {(int)bodies.size(), earth_mass * 1e8, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
    bodies.push_back(central_body);

    double max_coord = std::accumulate(
            bodies.begin(),
            bodies.end(),
            0.0,
            [&bodies](double acc, const Body &body) -> double
            {
                return std::max({acc, body.location[0], body.location[1], body.location[2]});
            }
    );

    const double delta_t = 1.0 / 30;
    const double max_time = 10;
    BarnesHut bh(bodies, delta_t, max_time);

    std::ofstream outp("data/simulation.txt", std::ofstream::out);
    outp << delta_t << "\n" << max_time << "\n" << n_bodies << "\n";

    time_point<system_clock> before = system_clock::now();
    bh.simulate([&outp](std::vector<Body> const &bodies) -> void
                {
                    for (Body body : bodies) {
                        outp << body << "\n";
                    }
                });
    time_point<system_clock> after = system_clock::now();
    double seconds = duration_cast<milliseconds>(after - before).count() / 1000;
    std::cout << seconds << " seconds\n";
}

void test_shit_sfml()
{
    using std::chrono::system_clock;
    using std::chrono::time_point;
    using std::chrono::milliseconds;
    using std::chrono::nanoseconds;
    using std::chrono::duration_cast;
    using std::chrono::duration;

    sf::RenderWindow window(sf::VideoMode(1000, 1000), "SFML works!");
    float cur_x = 0, cur_y = 0;
    sf::CircleShape shape(5);
    shape.setOrigin(cur_x, cur_y);
    shape.setFillColor(sf::Color::Green);

    time_point<system_clock> prev_time = system_clock::now();
    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed) {
                window.close();
            }
        }

        auto curr_time = system_clock::now();
        if (duration_cast<nanoseconds>(curr_time - prev_time).count() >= 16666666) {
            prev_time = curr_time;
            cur_x -= 1;
            cur_y -= 1;
            shape.setOrigin(cur_x, cur_y);
        }

        window.clear();
        window.draw(shape);
        window.display();
    }
}

int main()
{
    test_shit_5();
//    test_shit_sfml();

//    Vector3d x = Vector3d::Random(3, 1);
//    std::cout << 1000 * x << "\n";
}
