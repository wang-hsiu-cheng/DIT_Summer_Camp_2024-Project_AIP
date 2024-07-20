/*
 * path_vector_def.h
 *
 *  Created on: Jul 19, 2024
 *      Author: tu
 */

#ifndef INC_PATH_VECTOR_DEF_H_
#define INC_PATH_VECTOR_DEF_H_

#include <vector>

struct point
{
	double x = 0.0;
	double y = 0.0;
	double w = 0.0;
};

extern std::vector<point> path;
extern std::vector<point> obsticals;

#endif /* INC_PATH_VECTOR_DEF_H_ */
