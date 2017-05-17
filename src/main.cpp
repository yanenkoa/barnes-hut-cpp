#include <iostream>
#include <array>
#include <memory>
#include "Eigen/Eigen"

using namespace Eigen;

struct Body
{
    double mass;
    Vector3d point;

    Body(double mass_, Vector3d point_): mass(mass_), point(point_) {};
};

std::ostream & operator<<(std::ostream & os, Body & body)
{
    os << "{ mass = " << body.mass
       << ", point = { "
       << body.point[0] << ", "
       << body.point[1] << ", "
       << body.point[2]
       << " } }";
    return os;
}

class OcNode
{
private:
    double get_x_min(const OcNode & parent, std::size_t index)
    {
        if (index == 0 || index == 3 || index == 4 || index == 7) {
            return parent.x_min;
        } else {
            return (parent.x_max + parent.x_min) / 2;
        }
    }

    double get_x_max(const OcNode & parent, std::size_t index)
    {
        if (index == 0 || index == 3 || index == 4 || index == 7) {
            return (parent.x_max + parent.x_min) / 2;
        } else {
            return parent.x_max;
        }
    }

    double get_y_min(const OcNode & parent, std::size_t index)
    {
        if (index == 0 || index == 1 || index == 4 || index == 5) {
            return parent.y_min;
        } else {
            return (parent.y_max + parent.y_min) / 2;
        }
    }

    double get_y_max(const OcNode & parent, std::size_t index)
    {
        if (index == 0 || index == 1 || index == 4 || index == 5) {
            return (parent.y_max + parent.y_min) / 2;
        } else {
            return parent.y_max;
        }
    }

    double get_z_min(const OcNode & parent, std::size_t index)
    {
        if (index == 0 || index == 1 || index == 2 || index == 3) {
            return parent.z_min;
        } else {
            return (parent.z_max + parent.z_min) / 2;
        }
    }

    double get_z_max(const OcNode & parent, std::size_t index)
    {
        if (index == 0 || index == 1 || index == 2 || index == 3) {
            return (parent.z_max + parent.z_min) / 2;
        } else {
            return parent.z_max;
        }
    }

    Vector3d get_centerpoint()
    {
        return {
                (x_min + x_max) / 2,
                (y_min + y_max) / 2,
                (z_min + z_max) / 2
        };
    }

    std::size_t get_child_index(const Vector3d & point)
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

    bool is_internal = false;
    bool is_empty = true;
    Body center_of_mass = { 0, { 0, 0, 0 } };
    std::vector<std::shared_ptr<Body>> bodies = {};

    friend int main();

public:
    std::array<std::shared_ptr<OcNode>, 8> children;
    const double x_min, x_max, y_min, y_max, z_min, z_max;
    const Vector3d centerpoint;

    OcNode(
            const double x_min_,
            const double x_max_,
            const double y_min_,
            const double y_max_,
            const double z_min_,
            const double z_max_
    )
            : x_min(x_min_)
            , x_max(x_max_)
            , y_min(y_min_)
            , y_max(y_max_)
            , z_min(z_min_)
            , z_max(z_max_)
            , centerpoint(get_centerpoint())
    {
        std::cout << "creating new root\n";

    }

    OcNode(
            const OcNode & parent_,
            std::size_t index
    )
            : x_min(get_x_min(parent_, index))
            , x_max(get_x_max(parent_, index))
            , y_min(get_y_min(parent_, index))
            , y_max(get_y_max(parent_, index))
            , z_min(get_z_min(parent_, index))
            , z_max(get_z_max(parent_, index))
            , centerpoint(get_centerpoint())
    {
        std::cout << "creating new child\n";
    }

    void insert_point(const std::shared_ptr<Body> & body)
    {
        if (!is_empty) {
            is_internal = true;
            std::size_t curr_body_child_index = get_child_index(bodies[0]->point);
            children[curr_body_child_index].reset(new OcNode(*this, curr_body_child_index));
            children[curr_body_child_index]->insert_point(bodies[0]);
        } else {
            is_empty = false;
        }

        bodies.push_back(std::shared_ptr<Body>(body));
        double mass_prev = center_of_mass.mass;
        Vector3d point_prev = center_of_mass.point;
        double mass_new = mass_prev + body->mass;
        Vector3d point_new = (point_prev * mass_prev + body->point) / mass_new;
        center_of_mass = { mass_new, point_new };

        if (!is_internal) {
            return;
        }

        std::size_t child_index = get_child_index(body->point);
        if (children[child_index] == nullptr) {
            children[child_index].reset(new OcNode(*this, child_index));
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
    OcNode node(0, 4, 0, 4, 0, 4);
    std::shared_ptr<Body> b1(new Body(1, { 0.7, 0.7, 0.7 }));
    std::shared_ptr<Body> b2(new Body(1, { 0.2, 0.2, 0.2 }));

    node.insert_point(b1);
    node.insert_point(b2);

    for (auto body : node.children[0]->children[0]->children[6]->bodies) {
        std::cout << *body << "\n";
    }

/*
    std::cout << node.get_child_index({ 1, 1, 1 }) << "\n";
    std::cout << node.get_child_index({ 3, 1, 1 }) << "\n";
    std::cout << node.get_child_index({ 3, 3, 1 }) << "\n";
    std::cout << node.get_child_index({ 1, 3, 1 }) << "\n";
    std::cout << node.get_child_index({ 1, 1, 3 }) << "\n";
    std::cout << node.get_child_index({ 3, 1, 3 }) << "\n";
    std::cout << node.get_child_index({ 3, 3, 3 }) << "\n";
    std::cout << node.get_child_index({ 1, 3, 3 }) << "\n";
*/
}
