#pragma once

#include <vector>
#include <cassert>
#include <sstream>

#include "pugixml.hpp"
#include "pugi_tools.h"

#include <Eigen/Dense>


#define COMMON_EPS 1e-9

namespace tracking_tools
{
    template<class T>
    Eigen::MatrixXd make_matrix(std::size_t m, std::size_t n, std::vector<T> &input_v) {
        std::size_t size = input_v.size();
        if (m * n != size || m * n == 0) {
            throw std::runtime_error("Cannot make matrix: bad vector size");
        }

        Eigen::MatrixXd result_matrix = Eigen::Map<Eigen::MatrixXd>(input_v.data(), n, m);

        return result_matrix.transpose();
    }

    template<class T>
    Eigen::VectorXd make_vector(std::vector<T> &input_v) {
        std::size_t size = input_v.size();
        if (size == 0) {
            throw std::runtime_error("Cannot make vector: bad vector size");
        }

        Eigen::VectorXd result_vector = Eigen::Map<Eigen::VectorXd>(input_v.data(), size);

        return result_vector;
    }

    inline Eigen::MatrixXd construct_matrix(const pugi::xml_node &node) {
        pugi::xml_child child(node);

        auto M = atoi(child("M").child_value());
        auto N = atoi(child("N").child_value());

        assert(M > 0);
        assert(N > 0);

        std::string values = child("values").child_value();

        std::vector<double> v_values;

        std::stringstream ssin(values);

        double tmp;
        while (ssin >> tmp) {
            v_values.push_back(tmp);
        }

        Eigen::MatrixXd result_matrix;
        if (!v_values.empty()) {
            result_matrix = make_matrix(M, N, v_values);
        } else {
            result_matrix = Eigen::MatrixXd::Zero(M, N);
        }

        return result_matrix;
    }

    inline Eigen::VectorXd construct_vector(const pugi::xml_node &node) {
        pugi::xml_child child(node);

        auto M = atoi(child("M").child_value());
        auto N = atoi(child("N").child_value());
        std::string values = child("values").child_value();

        assert(M > 0);
        assert(N > 0);

        std::vector<double> v_values;

        std::stringstream ssin(values);
        double tmp;
        while (ssin >> tmp) {
            v_values.push_back(tmp);
        }

        Eigen::VectorXd result_vector;
        if (!v_values.empty()) {
            result_vector = make_vector(v_values);
        } else {
            result_vector = Eigen::VectorXd::Zero(M * N);
        }

        return result_vector;
    }
}