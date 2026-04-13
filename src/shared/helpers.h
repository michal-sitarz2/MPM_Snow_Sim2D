#pragma once

#include "Parameters.h"

#include <glm/glm.hpp>
#include <Eigen/Dense>
#include <cmath>
#include <iostream>

inline glm::mat2 VectorOuterProduct(glm::vec2 v, glm::vec2 d)
{
	return glm::mat2(
		v.x * d.x,   // col0, row0
		v.y * d.x,   // col0, row1
		v.x * d.y,   // col1, row0
		v.y * d.y    // col1, row1
	);
}

inline Eigen::Matrix2f GLMToEigen(glm::mat2 A)
{
	Eigen::Matrix2f result;
	result(0, 0) = A[0][0]; result(0, 1) = A[1][0];  // row 0
	result(1, 0) = A[0][1]; result(1, 1) = A[1][1];  // row 1
	return result;
}

inline glm::mat2 EigenToGLM(Eigen::Matrix2f A)
{
	return glm::mat2(
		A(0, 0), A(1, 0),   // col 0
		A(0, 1), A(1, 1)    // col 1
	);   
}