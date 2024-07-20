/*
 * path.cpp
 *
 *  Created on: Jul 17, 2024
 *      Author: tu
 */

#include "path.h"

bool global_planner_on = false;
bool look_ahead_dis_on = false;
std::vector<point> path;
std::vector<point> obsticals;

// Do path once
void do_path()
{
	point buffer;

	// ** Script
	// ------------------------------------------
	//		buffer = {0.8, 0.55, 0.0};	// ** Modify path points
	//		path.push_back(buffer);
	//		buffer = {1.8, 1.15, 0.0};	// ** Modify path points
	//		path.push_back(buffer);
	//		buffer = {1.3, 0.35, 0.0};	// ** Modify path points
	//		path.push_back(buffer);
	//		buffer = {0.8, 1.15, 0.0};	// ** Modify path points
	//		path.push_back(buffer);
	//		buffer = {1.8, 0.55, 0.0};	// ** Modify path points
	//		path.push_back(buffer);
	//		buffer = {1.3, 1.35, 0.0};	// ** Modify path points
	//		path.push_back(buffer);
	//		buffer = {0.0, 0.0, 0.0};	// ** Modify path points
	//		path.push_back(buffer);
	buffer = {0.2, 0.2, 0.0}; // ** Modify path points
	path.push_back(buffer);
	buffer = {0.7, 0.7, 0.0}; // ** Modify path points
	path.push_back(buffer);
	buffer = {1.5, 1.5, 0.0}; // ** Modify path points
	path.push_back(buffer);

	//		buffer = {0.0, 0.0, 0.0};	// ** Modify path points
	//		path.push_back(buffer);
	//		buffer = {0.0, 0.4, 0.0};	// ** Modify path points
	//		path.push_back(buffer);
	//		buffer = {0.0, 0.0, 0.0};	// ** Modify path points
	//		path.push_back(buffer);
	//	path.push_back(buffer);

	// ** Obstical setup
	buffer = {0.4, 0.4, 0.05};
	obsticals.push_back(buffer);
	buffer = {1.0, 1.0, 0.05};
	obsticals.push_back(buffer);
	// ------------------------------------------

	if (global_planner_on)
		return; // TODO: Global planner processing
	if (look_ahead_dis_on)
		return; // TODO: Check for finish
	else
	{
		while (1)
		{
			if (done)
				break;
		}
		return;
	}
}
