#include <iostream>
#include <array>
#include <memory>
#include "Eigen/Eigen"

using namespace Eigen;

struct Body
{
    double mass;
    Vector3d point;

    Body(double mass_, Vector3d point_) : mass(mass_), point(point_)
    {};
};

std::ostream &operator<<(std::ostream &os, Body &body)
{
    os << "{ mass = " << body.mass
       << ", point = { "
       << body.point[0] << ", "
       << body.point[1] << ", "
       << body.point[2]
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

    NodeType type = EMPTY_EXTERNAL;
    Body center_of_mass = {0, {0, 0, 0}};

    friend int main();

public:
    std::array<std::shared_ptr<OcNode>, 8> children{
            nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr
    };
    const Vector3d lower_vertice, upper_vertice;
    const Vector3d centerpoint;

    OcNode(
            const Vector3d &lower_vertice_,
            const Vector3d &upper_vertice_
    ) : lower_vertice(lower_vertice_), upper_vertice(upper_vertice_), centerpoint((lower_vertice + upper_vertice) / 2)
    {}

    void insert_point(const Body &body)
    {
        if (type == NONEMPTY_EXTERNAL) {
            type = INTERNAL;
            auto i = get_child_index(center_of_mass.point);
            std::size_t curr_body_child_index = get_child_index(center_of_mass.point);
            children[curr_body_child_index].reset(new OcNode(
                    get_child_lower_vertice(curr_body_child_index),
                    get_child_upper_vertice(curr_body_child_index)
            ));
            children[curr_body_child_index]->insert_point(center_of_mass);
        }

        double mass_prev = center_of_mass.mass;
        Vector3d point_prev = center_of_mass.point;
        double mass_new = mass_prev + body.mass;
        Vector3d point_new = (point_prev * mass_prev + body.point) / mass_new;
        center_of_mass = {mass_new, point_new};

        if (type == EMPTY_EXTERNAL) {
            type = NONEMPTY_EXTERNAL;
            return;
        }

        std::size_t child_index = get_child_index(body.point);
        if (children[child_index] == nullptr) {
            children[child_index].reset(new OcNode(
                    get_child_lower_vertice(child_index),
                    get_child_upper_vertice(child_index)
            ));
        }
        children[child_index]->insert_point(body);
    }
};

class OcTree
{
private:


public:


};


int main()
{
    OcNode node({0, 0, 0}, {4, 4, 4});

    std::vector<Vector3d> norm_shifts {
            {0, 0, 0},
            {1, 0, 0},
            {1, 1, 0},
            {0, 1, 0},
            {0, 0, 1},j
            {1, 0, 1},
            {1, 1, 1},
            {0, 1, 1},
    };

    double cross_coef = 2;

    for (Vector3d norm_shift : norm_shifts) {
        node.insert_point({1, Vector3d({0.7, 0.7, 0.7}) + norm_shift * cross_coef});
    }

    for (std::size_t i = 0; i < 8; ++i) {
        for (std::size_t j = 0; j < 8; ++j) {
            assert(node.children[i]->get_child_index(
                    (norm_shifts[j] * cross_coef + Vector3d(1, 1, 1)) / 2 + norm_shifts[i] * cross_coef
            ) == j);
        }
    }
}
