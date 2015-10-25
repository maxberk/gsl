/*
Graph Search Library
Copyright(c) 2000-2002 

Pathfinding thru Waypoints sample.
*/

#if defined(_MSC_VER)
#ifdef _DEBUG
#pragma warning( disable : 4786 )
#endif
#endif

// STL
#include <map>
#include <functional>
#include <algorithm>
#include <math.h>

// GSL files
#include "config.h" 
#include "traverser.h"
#include "solver.h"
#include "prnpath.h"

#define SQR(x) (x)*(x)

/* Waypoint structure */
struct waypoint {
	static long nextid; // last id assigned to created waypoint
	long id; // id of waypoint
	float x, y; // x and y coordinates
	waypoint(float ax, float ay) : x(ax), y(ay) { id = nextid++; }
};
long waypoint::nextid = 0;

// how routes' map with waypoints is stored
typedef vector<waypoint> waypoints_t;
typedef waypoints_t::const_iterator waypoints_const_iter_t;
waypoints_t waypoints;

// 1st parameter for a traverser - description of a node in a search tree
// it is description of a route to some waypoint and includes a terminal waypoint
// and an ordered list (from beginning to end) of intermediate waypoints on this route
struct node_value {
	long waypointid; // a terminal waypoint
	vector<long> previds; // list of intermediate waypoints from a route beginning to an end
	// operator < is used in ordering operations
	bool operator <(const node_value& other) const {
		return waypointid < other.waypointid;
	}
	// operator << is used to transform a node value to representation on a display
	friend ostream& operator <<(ostream& os, const node_value& other) {
		return os << other.waypointid;
	}
	bool operator ==(const node_value& other) const {
		bool result = (waypointid == other.waypointid);
		if (result) {
			result = (previds.size() == other.previds.size());
		}
		if (result) {
			result = equal(previds.begin(), previds.end(), other.previds.begin());
		}
		return result;
	}
	node_value& operator =(const node_value &other) {
		waypointid = other.waypointid;
		previds.clear();
		copy(other.previds.begin(), other.previds.end(),
			inserter(previds, previds.begin()));
		return *this;
	}
	// Constructor for root node
	node_value(long awaypointid = 0) : waypointid(awaypointid) {}
	// Constructor for regular node - new id and parent node
	node_value(long awaypointid, node_value& prev) : waypointid(awaypointid) {
		// create an ordered list of intermediate waypoints
		copy(prev.previds.begin(), prev.previds.end(),
			inserter(previds, previds.begin()));
		previds.push_back(prev.waypointid);
	}
};

// 2nd parameter for a traverser - description of a node cost in a search tree
// it consists of two components - summarized cost to a node and a heuristics
// that evaluates cost up to a terminal node
struct cost_value {
		// two components of cost
		float g, h;
		// operators < ...
		bool operator <(const cost_value& other) const {
			return (g + h) < (other.g + other.h);
		}
		// ... and < are used in a traverser code orderinf operations for further node selection
		bool operator >(const cost_value& other) const {
			return (g + h) > (other.g + other.h);
		}
		// string representation of a cost value
		friend ostream& operator <<(ostream& os, const cost_value& other) {
			return os << (other.g + other.h);
		}
		// constructors
		cost_value(float ag, float ah) : g(ag), h(ah) {}
		cost_value(float ag = 0) : g(ag), h(0) {}
};

struct obstacle {
	static long nextid; // last id assigned to created obstacle
	long id; // id of obstacle 
	float x, y; // coordinates of center
	float r; // radius value;
	obstacle(float ax, float ay, float ar) : x(ax), y(ay), r(ar) {}
};
long obstacle::nextid = 0;

// how obstacles are stored
typedef vector<obstacle> obstacles_t;
typedef obstacles_t::const_iterator obstacles_const_iter_t;
obstacles_t obstacles;

// 3rd parameter for a traverser - customized expansion function
class expand {
	// maximal lag length
	float _MaxLagLen;
public:
	// main operator that is used according to GSL specification
	// it is used as a callback from a traverser code
	// list<node_value> &e - list of child nodes - this list is used as an output - the follwoing code
	// pushes node values to this list
	// list<cost_value>& ce - output list for costs of child nodes - costs should have the same
	// positions at the list as corresponding nodes in a previous list
	// node_value n - this is a node to be expanded
	// node_value - this is a parent of a previous node
	// cost_value c - the cost of a node to be expanded
	void operator()(list<node_value> &e, list<cost_value>& ce, node_value n, node_value, cost_value c) {

		// x and y coordinates of a lag start waypoint
		float xs = waypoints[n.waypointid].x;
		float ys = waypoints[n.waypointid].y;

		// iterator over all waypoints on a routes' map
		waypoints_const_iter_t i = waypoints.begin();
		for(;i != waypoints.end();++ i) {
			waypoint wp = *i;

			// find if a waypoint is already examined in a current route - to avoid cycles
			vector<long>::iterator i = find(
				n.previds.begin(),n.previds.end(),wp.id);
			if( (i == n.previds.end()) && (wp.id != n.waypointid) ) {

				// x and y coordinates of a lag end waypoint
				float xf = wp.x;
				float yf = wp.y;

				// lag length
				float LagLen = sqrt( SQR(xf - xs) + SQR(yf - ys) );

				// waypoint is added if the lag length does not exceed predefined value
				if(LagLen < _MaxLagLen) {
					bool crosses = false;
					// iterating over all obstacles
					for(obstacles_const_iter_t j = obstacles.begin(); j != obstacles.end(); ++j) {
						obstacle ob = *j;
						float xo = ob.x;
						float yo = ob.y;
						float ro = ob.r;
						
						float a = SQR( (xf - xs)/(yf - ys) ) + 1.0;
						float b = 2.0 * (xf - xs)/(yf - ys) * ( (xs - xo) - (xf - xs)/(yf - ys) * ys ) - 2.0 * yo;
					        float c = SQR( (xs - xo) - (xf - xs)/(yf - ys) * ys ) + SQR(yo) - SQR(ro);
						float D = SQR(b) - 4.0 * a * c;
						// if a line connecting two waypoints crosses an obstacle area
						if(D > 0.0) {
							// find whether the area is crossed between waypoints
							float y1 = ( - b + sqrt(D) ) / (2.0 * a);
							float y2 = ( - b - sqrt(D) ) / (2.0 * a);
							float ymin = ys > yf? yf: ys;
							float ymax = ys > yf? ys: yf;
							if ( (y1 <= ymax) && (y1 >= ymin) || (y2 <= ymax) && (y2 >= ymin) ) {
								crosses = true;
								break;
							}
						}

					}

					if(!crosses) {
						// adding node value to a result list
						e.push_back(node_value(wp.id, n));
	
						// calculation of heuristics
						// x and y coordinates of a final route waypoint
						float xg = waypoints[19].x;
						float yg = waypoints[19].y;

						// euclidean distance for a child node waypoint to final route waypoint
						float h = sqrt( SQR(xf - xg) + SQR(yf - yg) );
						// adding cost value to a result list
						// one should put 0.0 instead of h as a constrcutor parameter here
						// if no heuristics is used
						ce.push_back(cost_value(c.g + LagLen, h));
					}
				}
			}
		}
	}
	float getMaxLagLen() const { return _MaxLagLen; }
	void setMaxLagLen(float value) { _MaxLagLen = value; }
	expand() : _MaxLagLen(1000.0) {}

};

// these two functions are used to make callbacks from a traverser code when
// "expand root node" and "select cursor" events occur.
long num_expansions;
// initialization of number of expansions counter
void OnExpandRoot() { num_expansions = 0; }
// this function is called when a node is selected for expansion
void OnSelectCursor(const node_value&, const cost_value&) { ++num_expansions; }

// goal predicate - is used to terminate search process
bool goal(node_value n) { return n.waypointid == 19; }

int main()
{
	// push waypoints to a routes' map
	waypoints.push_back(waypoint(-4.0, -3.0)); // 0
	waypoints.push_back(waypoint(-3.0, -1.0)); // 1
	waypoints.push_back(waypoint(1.0, -4.0)); // 2
	waypoints.push_back(waypoint(3.0, -2.0)); // 3
	waypoints.push_back(waypoint(-1.0, 1.0)); // 4
	waypoints.push_back(waypoint(4.0, -3.0)); // 5
	waypoints.push_back(waypoint(2.0, 1.0)); // 6
	waypoints.push_back(waypoint(-2.0, 2.0)); // 7
	waypoints.push_back(waypoint(5.0, -1.0)); // 8
	waypoints.push_back(waypoint(6.0, 2.0)); // 9
	waypoints.push_back(waypoint(-1.0, 4.0)); // 10
	waypoints.push_back(waypoint(4.0, 4.0)); // 11
	waypoints.push_back(waypoint(8.0, 3.0)); // 12
	waypoints.push_back(waypoint(1.0, 6.0)); // 13
	waypoints.push_back(waypoint(6.0, 5.0)); // 14
	waypoints.push_back(waypoint(2.0, 7.0)); // 15
	waypoints.push_back(waypoint(7.0, 7.0)); // 16
	waypoints.push_back(waypoint(4.0, 7.0)); // 17
	waypoints.push_back(waypoint(6.0, 9.0)); // 18
	waypoints.push_back(waypoint(8.0, 9.0)); // 19

	obstacles.push_back(obstacle(0.0,5.0,1.0)); // 0
	obstacles.push_back(obstacle(3.0,3.0,2.0)); // 1
	obstacles.push_back(obstacle(3.5,0.0,1.0)); // 2
	obstacles.push_back(obstacle(8.0,7.0,2.5)); // 3

	// define exponential traverser type and create an exemplar
	typedef ExpTr<node_value,cost_value,expand,less<cost_value> > ExpTr_t;
	ExpTr_t j;

	cout << "in exp traverser forward search ...\n";
	j = ExpTr_t(0);
	// set a maximal lag length
	j.getexpfunc().setMaxLagLen(5.0); // or 6.0
	// add two defined handlers to enable calculation of an expansions number
	j.set_handler_on_expand_root(OnExpandRoot);
	j.set_handler_on_select_cursor(OnSelectCursor);
	// launch A* algorithm (combination of a predefined heuristics with a best - first
	// search that is provided by exponential traverser is A*!)
	ForwardSearch(j, goal);
	ShowPath(j);
	
	// show founded path
	cout << "number of expansions = " << num_expansions << '\n';
	cout << "...out \n\n";

	cout << "Search is completed.\n";

	return 0;

}
