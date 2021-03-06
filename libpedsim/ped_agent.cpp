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
#include <iostream>

Ped::Tagent::Tagent(int posX, int posY) {
  isinitialised = false;
  Ped::Tagent::init(posX, posY);
}

Ped::Tagent::Tagent(double posX, double posY) {
  isinitialised = false;
  Ped::Tagent::init((int)round(posX), (int)round(posY));
}

void Ped::Tagent::initPointers(int i, int *ax, int *ay, 
			       int *dx, int *dy, 
			       int *desix, int *desiy) {
  isinitialised = true;
  index = i;
  arr_x = ax;
  arr_y = ay;

  arr_desiredPositionX = dx;
  arr_desiredPositionY = dy;
  arr_destinationX = desix;
  arr_destinationY = desiy;
  
  // Make sure all values are initialised
  arr_desiredPositionX[index] = 0;
  arr_desiredPositionY[index] = 0;
  arr_x[index] = x;
  arr_y[index] = y;
  destination = NULL;
}

void Ped::Tagent::init(int posX, int posY) {
  x = posX;
  y = posY;
}

/*
  COMPUTE NEXT DESIRED POSITION
  diffx = dest.x - x
  diffy = dest.y - y
  len = sqrt( diffx * diffx + diffy * diffy )
  desix = round(x + diffx / len)
  desiy = round(y + diffy / len)
 */

void Ped::Tagent::computeNextDesiredPosition() {
  destination = getNextDestination();

  // Update values
  arr_destinationX[index] = destination->getx();
  arr_destinationY[index] = destination->gety();

  if (destination == NULL) {
    // no destination, no need to
    // compute where to move to
    return;
  }
  
  double diffX = arr_destinationX[index] - arr_x[index];
  double diffY = arr_destinationY[index] - arr_y[index];
  double len = sqrt(diffX * diffX + diffY * diffY);
  arr_desiredPositionX[index] = (int) round(arr_x[index] + diffX / len);
  arr_desiredPositionY[index] = (int) round(arr_y[index] + diffY / len);
}

void Ped::Tagent::addWaypoint(Twaypoint* wp) {
  waypoints.push_back(wp);
}

Ped::Twaypoint* Ped::Tagent::getNextDestination() {
  Ped::Twaypoint* nextDestination = NULL;
  bool agentReachedDestination = false;
  
  if (destination != NULL) {
    // compute if agent reached its current destination
    double diffX = arr_destinationX[index] - arr_x[index];
    double diffY = arr_destinationY[index] - arr_y[index];
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
