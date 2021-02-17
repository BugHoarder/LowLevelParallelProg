//
// pedsim - A microscopic pedestrian simulation system.
// Copyright (c) 2003 - 2014 by Christian Gloor
//
// Adapted for Low Level Parallel Programming 2017
//
// TAgent represents an agent in the scenario. Each
// agent has a position (x,y) and a number of destinations
// it wants to visit (waypoints). The desired next position
// represents the position it would like to visit next as it
// will bring it closer to its destination.
// Note: the agent will not move by itself, but the movement
// is handled in ped_model.cpp. 
//

#ifndef _ped_agent_h_
#define _ped_agent_h_ 1

#include <vector>
#include <deque>

using namespace std;

namespace Ped {
        class Twaypoint;

	class Tagent {
	public:
		Tagent(int posX, int posY);
		Tagent(double posX, double posY);

		void initPointers(int i, 
				  vector<int> *x, vector<int> *y, 
				  vector<int> *dx, vector<int> *dy, 
				  vector<Twaypoint*> *des, 
				  vector<Twaypoint*> *ldes);

		// Returns the coordinates of the desired position
		int getDesiredX() const { return arr_desiredPositionX->at(index); }
		int getDesiredY() const { return arr_desiredPositionY->at(index); }

		// Sets the agent's position
		void setX(int newX) { arr_x->at(index) = newX; }
		void setY(int newY) { arr_y->at(index) = newY; }

		// Update the position according to get closer
		// to the current destination
		void computeNextDesiredPosition();

		// Position of agent defined by x and y
		int getX() const { return arr_x->at(index); };
		int getY() const { return arr_y->at(index); };

		// Adds a new waypoint to reach for this agent
		void addWaypoint(Twaypoint* wp);

	private:
		vector<int> *arr_x;
		vector<int> *arr_y;
		vector<int> *arr_desiredPositionX;
		vector<int> *arr_desiredPositionY;
		vector<Twaypoint*> *arr_destination;
		vector<Twaypoint*> *arr_lastDestination;
		//vector<deque<Twaypoint*>> *arr_waypoints;

		Tagent() {};

		int index; // position in vectors

		// The agent's current position
		int x;
		int y;

		// The agent's desired next position
		//int desiredPositionX;
		//int desiredPositionY;

		// The current destination (may require several steps to reach)
		Twaypoint* destination;

		// The last destination
		Twaypoint* lastDestination;

		// The queue of all destinations that this agent still has to visit
		deque<Twaypoint*> waypoints;

		// Internal init function 
		void init(int posX, int posY);

		// Returns the next destination to visit
		Twaypoint* getNextDestination();
	};
}

#endif
