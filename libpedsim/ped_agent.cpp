//
// pedsim - A microscopic pedestrian simulation system.
// Copyright (c) 2003 - 2014 by Christian Gloor
//
//
// Adapted for Low Level Parallel Programming 2017
//
#include "ped_agent.h"
#include "ped_waypoint.h"
#include <math.h>

#include <stdlib.h>

Ped::Tagent::Tagent(int posX, int posY) {
	Ped::Tagent::init(posX, posY);
}

Ped::Tagent::Tagent(double posX, double posY) {
	Ped::Tagent::init((int)round(posX), (int)round(posY));
}

void Ped::Tagent::initPointers(int i, vector<int> *ax, vector<int> *ay, 
			       vector<int> *dx, vector<int> *dy, 
			       vector<Twaypoint*> *des, vector<Twaypoint*> *ldes) {
  index = i;
  arr_x = ax;
  arr_y = ay;
  arr_desiredPositionX = dx;
  arr_desiredPositionY = dy;
  arr_destination = des;
  arr_lastDestination = ldes;
  //arr_waypoints = wp;

  arr_x->at(index) = x;
  arr_y->at(index) = y;
}

void Ped::Tagent::init(int posX, int posY) {
  arr_x->at(index) = posX;
  arr_y->at(index) = posY;
  arr_destination->at(index) = NULL;
  arr_lastDestination->at(index) = NULL;
}

void Ped::Tagent::computeNextDesiredPosition() {
  destination = getNextDestination();
  if (destination == NULL) {
    // no destination, no need to
    // compute where to move to
    return;
  }
  
  double diffX = arr_destination->at(index)->getx() - arr_x->at(index);
  double diffY = arr_destination->at(index)->gety() - arr_y->at(index);
  double len = sqrt(diffX * diffX + diffY * diffY);
  arr_desiredPositionX->at(index) = (int) round(arr_x->at(index) + diffX / len);
  arr_desiredPositionY->at(index) = (int) round(arr_y->at(index) + diffY / len);
}

void Ped::Tagent::addWaypoint(Twaypoint* wp) {
  waypoints.push_back(wp);
}

Ped::Twaypoint* Ped::Tagent::getNextDestination() {
  Ped::Twaypoint* nextDestination = NULL;
  bool agentReachedDestination = false;
  
  if (destination != NULL) {
    // compute if agent reached its current destination
    double diffX = arr_destination->at(index)->getx() - arr_x->at(index);
    double diffY = arr_destination->at(index)->gety() - arr_y->at(index);
    double length = sqrt(diffX * diffX + diffY * diffY);
    agentReachedDestination = length < destination->getr();
  }
  
  if ((agentReachedDestination || destination == NULL) && !waypoints.empty()) {
    // Case 1: agent has reached destination (or has no current destination);
    // get next destination if available
    waypoints.push_back(destination);
    nextDestination = waypoints.front();
    waypoints.pop_front();
  }
  else {
    // Case 2: agent has not yet reached destination, continue to move towards
    // current destination
    nextDestination = destination;
  }

  return nextDestination;
}
