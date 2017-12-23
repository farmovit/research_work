#pragma once

#include <vector>

#include <Eigen/Dense>


#define COMMON_EPS 1e-9


template <class T>
Eigen::MatrixXd makeMatrix(std::size_t m, std::size_t n, std::vector<T>& input_v)
{
	std::size_t size = input_v.size();
	if( m*n != size ) {
		throw std::runtime_error("Cannot make matrix: bad vector size");
	}
	
	Eigen::MatrixXd result_matrix = Eigen::Map<Eigen::MatrixXd>(input_v.data(), n, m);
	
	return result_matrix.transpose();
}

template <class T>
Eigen::MatrixXd makeVector(std::vector<T>& input_v)
{
	std::size_t size = input_v.size();
	if( size == 0 ) {
		throw std::runtime_error("Cannot make vector: bad vector size");
	}
	
	Eigen::VectorXd result_vector = Eigen::Map<Eigen::VectorXd>(input_v.data(), size);
	
	return result_vector;
}