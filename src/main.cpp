#include <iostream>
#include <array>
#include <memory>
#include <queue>
#include <random>
#include "Eigen/Eigen"

using namespace Eigen;

struct Body
{
    double mass;
    Vector3d location;
    Vector3d velocity;
    Vector3d acceleration;
};

std::ostream &operator<<(std::ostream &os, Body &body)
{
    os << "{ mass = " << body.mass
       << ", point = { "
       << body.location[0] << ", "
       << body.location[1] << ", "
       << body.location[2]
       << " } }";
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

    Vector3d get_force(const Body &body1, const Body &body2)
    {
        Vector3d diff_vector = body1.location - body2.location;
        double dist = diff_vector.norm();
        double force_value = G * body1.mass * body2.mass / (dist * dist);
        return diff_vector.normalized() * force_value;
    }

    static constexpr double G = 6.67408e-11;
    static constexpr double theta = 0.5;

    NodeType type = EMPTY_EXTERNAL;
    Body center_of_mass = {0, {0, 0, 0}};

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
            : lower_vertice(lower_vertice_)
            , upper_vertice(upper_vertice_)
            , centerpoint((lower_vertice_ + upper_vertice_) / 2)
    {}

    void insert_point(const Body &body)
    {
        if (type == NONEMPTY_EXTERNAL) {
            type = INTERNAL;
            auto i = get_child_index(center_of_mass.location);
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
        Vector3d point_new = (point_prev * mass_prev + body.location) / mass_new;
        center_of_mass = {mass_new, point_new};

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
            case EMPTY_EXTERNAL:
                return {0, 0, 0};
            case NONEMPTY_EXTERNAL:
                return get_force(center_of_mass, body);
            case INTERNAL:
                double s = upper_vertice[0] - lower_vertice[0];
                double d = (center_of_mass.location - body.location).norm();
                if (s / d < theta) {
                    return get_force(center_of_mass, body);
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
    }
};

class OcTree
{
private:
    OcNode root;
    std::vector<Body> &bodies;
    const double delta_t;
    const double max_time;

public:
    OcTree(
            std::vector<Body> &bodies_,
            const Vector3d &lower_vertice,
            const Vector3d &upper_vertice,
            double delta_t_,
            double max_time_
    ) : bodies(bodies_), root(OcNode(lower_vertice, upper_vertice)), delta_t(delta_t_), max_time(max_time_) {}


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
            std::cout << 1;
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

    for (Vector3d norm_shift : norm_shifts) {
        node->insert_point({1, Vector3d({0.7, 0.7, 0.7}) + norm_shift * cross_coef});
    }
    node->insert_point({1, {0.2, 0.2, 0.2}});

    std::cout << OcNode::G << "\n";

    IOFormat CommaInitFmt(StreamPrecision, DontAlignCols, ", ", ", ", "", "", " ", ";");

    for (std::size_t i = 0; i < 8; ++i) {
        for (std::size_t j = 0; j < 8; ++j) {
            assert(node->children[i]->get_child_index(
                    (norm_shifts[j] * cross_coef + Vector3d(1, 1, 1)) / 2 + norm_shifts[i] * cross_coef
            ) == j);
        }

        std::cout << node->get_child_lower_vertice(i).format(CommaInitFmt) << " "
                  << node->get_child_upper_vertice(i).format(CommaInitFmt) << "\n";
    }

    print_shit(node);
}

int main()
{
    test_shit();
}
