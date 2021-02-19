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
				  int *x, int *y, 
				  int *dx, int *dy, 
				  int *desix, int *desiy);

		// Returns the coordinates of the desired position
		int getDesiredX() const { return arr_desiredPositionX[index]; }
		int getDesiredY() const { return arr_desiredPositionY[index]; }

		// Sets the agent's position
		void setX(int newX) { arr_x[index] = newX; }
		void setY(int newY) { arr_y[index] = newY; }

		// Update the position according to get closer
		// to the current destination
		void computeNextDesiredPosition();

		// Position of agent defined by x and y
		// These have been modified such that they return the packed values if initialised
		int getX() const { 
		  if (isinitialised)
		    return arr_x[index];
		  else
		    return x;
		};
		
		int getY() const { 
		  if (isinitialised) 
		    return arr_y[index];
		  else
		    return y;
		};

		// Adds a new waypoint to reach for this agent
		void addWaypoint(Twaypoint* wp);

	private:
		bool isinitialised;
		
		// Pointers to externally aligned data
		int *arr_x;
		int *arr_y;
		int *arr_desiredPositionX;
		int *arr_desiredPositionY;
		int *arr_destinationX;
		int *arr_destinationY;

		//vector<Twaypoint*> *arr_destination;
		//vector<Twaypoint*> *arr_lastDestination;
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
		//Twaypoint* lastDestination;

		// The queue of all destinations that this agent still has to visit
		deque<Twaypoint*> waypoints;

		// Internal init function 
		void init(int posX, int posY);

		// Returns the next destination to visit
		Twaypoint* getNextDestination();
	};
}

#endif
