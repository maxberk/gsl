/*
Graph Search Library
Copyright(c) 2000-2003 

Pathfinding on Hexagonal Grid sample.
*/

#if defined(_MSC_VER)
#ifdef _DEBUG
#pragma warning( disable : 4786 )
#endif
#endif

// STL
#include <map>
#include <functional>
#include <math.h>

// GSL files
#include "config.h" 
#include "traverser.h"
#include "solver.h"
#include "prnpath.h"

// length of a lag in a hexagonal grid
const float lag_len = 1.0;
// length of projections to X axis and...
const float lag_len_cos60 = lag_len * 0.5;
// ... to Y axis
const float lag_len_sin60 = lag_len * 0.866025;

// world size
// imin and imax are indices of left and right boundary vertices
long imin = 0;
long imax = 39;
// jmin and jmax are indices of upper and lower boundary vertices
long jmin = 0;
long jmax = 39;

// getX() is used to get an X value from grid coordinates
float getX(long i, long j) {
	float result = lag_len * i;
	if (j%2 != 0) {
		result -= lag_len_cos60;
	}
	return result;
}

// getY() is used to get an Y value from grid coordinates
float getY(long j) {
	return lag_len_sin60 * j;
}

// circle-based zone
struct zone {
	float xcenter, ycenter; // cooordinates of center
	float radius; // radius of zone
	bool contains(float x, float y) const {
		float dist = sqrt( (xcenter - x) * (xcenter - x) +
			(ycenter - y) * (ycenter - y) );
		return (dist < radius);
	}
	zone(float axcenter, float aycenter, float aradius):
		xcenter(axcenter), ycenter(aycenter), radius(aradius) {}
};

// list of zones
typedef list<zone> zones_t;
typedef zones_t::iterator zones_iter_t;
zones_t zones;

// map with obstacles: 0 - no obstacle, 1 - is obstacle
typedef map<long, map<long, int> > obstacles_t;
obstacles_t obstacles;

// node_value structure
// i and j grid coordinates of a node in a grid
struct node_value {
	long i, j;
	bool operator ==(const node_value& other) const {
		return ( (i==other.i) && (j == other.j) );
	}
	bool operator <(const node_value& other) const {
		if (i < other.i) {
			return true;
		} else if (i == other.i) {
			return (j < other.j);
		} else {
			return false;
		}
	}
	friend ostream& operator <<(ostream& os, const node_value& other) {
		return os << "(" << other.i << "," << other.j << ")";
	}
	node_value& operator =(const node_value &other) {
		i = other.i;
		j = other.j;
		return *this;
	}
	node_value(const node_value &other) : i(other.i), j(other.j) {}
	node_value(long ai = 0, long aj = 0) : i(ai), j(aj) { }
};

// list of goal nodes- points in a grid
typedef vector<node_value> goals_t;
typedef goals_t::const_iterator goals_const_iter_t;
goals_t goals;

// cost_value - the same as for waypoints.cpp sample
struct cost_value {
		float g, h;
		bool operator <(const cost_value& other) const {
			return (g + h) < (other.g + other.h);
		}
		bool operator >(const cost_value& other) const {
			return (g + h) > (other.g + other.h);
		}
		friend ostream& operator <<(ostream& os, const cost_value& other) {
			return os << (other.g + other.h);
		}
		cost_value(float ag, float ah) : g(ag), h(ah) {}
		cost_value(float ag = 0) : g(ag), h(0) {}
};

// node expansion function class
class expand {
private:
	// a grid node is added to an expanded list if
	// it is located inside a world and it is not over
	// an obstacle
	void add_child(list<node_value> &e, long i, long j) {
		if ((i > imax) || (i < imin) || (j > jmax) || (j < jmin) )
			return;
		if (obstacles[i][j] == 0) {
			e.push_back(node_value(i,j));
		}
	}
public:
	// important operator - get a node n, its parent p and
	// node cost c ->>> creates a list of succesors e with
	// a list of their cost values ce
	// after expansion a three possible directions of moves
	// are generated in a side opposite to a parent node -
	// the bot cannot change move direction immediately.
	void operator()(list<node_value> &e, list<cost_value>& ce, node_value n, node_value p, cost_value c) {
		long i = n.i;
		long j = n.j;
		long pi = p.i;
		long pj = p.j;
		// for two types of nodes - ...
		// first type
		if (j%2 != 0) { // this one
			// if node has no parent
			// parent is itself
			if( (pi == i) && (pj == j) ) {
				add_child(e,i-1,j);		
				add_child(e,i-1,j-1);		
				add_child(e,i,j-1);		
				add_child(e,i+1,j);		
				add_child(e,i,j+1);		
				add_child(e,i-1,j+1);		
			} else if( (pi == (i-1)) && (pj == j)) {
				add_child(e,i,j-1);
				add_child(e,i+1,j);
				add_child(e,i,j+1);
			} else if( (pi == (i-1)) && (pj == (j-1)) ) {
				add_child(e,i+1,j);
				add_child(e,i,j+1);
				add_child(e,i-1,j+1);
			} else if( (pi == i) && (pj == (j-1)) ) {
				add_child(e,i,j+1);
				add_child(e,i-1,j+1);
				add_child(e,i-1,j);
			} else if( (pi == (i+1)) && (pj == j) ) {
				add_child(e,i-1,j+1);
				add_child(e,i-1,j);
				add_child(e,i-1,j-1);
			} else if( (pi == i) && (pj == (j+1)) ) {
				add_child(e,i-1,j);
				add_child(e,i-1,j-1);
				add_child(e,i,j-1);
			} else if( (pi == (i-1)) && (pj == (j+1)) ) {
				add_child(e,i-1,j-1);
				add_child(e,i,j-1);
				add_child(e,i+1,j);
			}
		} else {
			// second type
			if( (pi == i) && (pj == j) ) {
				add_child(e,i-1,j);		
				add_child(e,i,j-1);		
				add_child(e,i+1,j-1);		
				add_child(e,i+1,j);		
				add_child(e,i+1,j+1);		
				add_child(e,i,j+1);		
			} else if( (pi == (i-1)) && (pj == j)) {
				add_child(e,i+1,j-1);
				add_child(e,i+1,j);
				add_child(e,i+1,j+1);
			} else if( (pi == i) && (pj == (j-1)) ) {
				add_child(e,i+1,j);
				add_child(e,i+1,j+1);
				add_child(e,i,j+1);
			} else if( (pi == (i+1)) && (pj == (j-1)) ) {
				add_child(e,i+1,j+1);
				add_child(e,i,j+1);
				add_child(e,i-1,j);
			} else if( (pi == (i+1)) && (pj == j) ) {
				add_child(e,i,j+1);
				add_child(e,i-1,j);
				add_child(e,i,j-1);
			} else if( (pi == (i+1)) && (pj == (j+1)) ) {
				add_child(e,i-1,j);
				add_child(e,i,j-1);
				add_child(e,i+1,j-1);
			} else if( (pi == i) && (pj == (j+1)) ) {
				add_child(e,i,j-1);
				add_child(e,i+1,j-1);
				add_child(e,i+1,j);
			}
		}


		// accumulated and heuristic cost
		// accumulated - total length odf lags in a route
		// heuristic - a minimal distance to a goal nodes' set
		for (list<node_value>::iterator n = e.begin(); n != e.end(); ++n) {
			i = (*n).i;
		       	j = (*n).j;

			float x = getX(i,j);
			float y = getY(j);
			float d = 10000000.0;
			for(goals_const_iter_t g = goals.begin(); g != goals.end(); ++g ) {
				float xgoal = getX((*g).i, (*g).j);
				float ygoal = getY((*g).j);
				float d1 = sqrt(
					(x - xgoal) * (x - xgoal) +
					(y - ygoal) * (y - ygoal)
				);
				if (d1 < d) {
					d = d1;
				}
			}
			ce.push_back(cost_value(c.g + lag_len, d)); 
		}
	}
	expand() {}

};

long num_expansions;
void OnExpandRoot() { num_expansions = 0; }
void OnSelectCursor(const node_value& n, const cost_value& c) {
	++num_expansions;
}

// goal function - if a grid node corresponds to one of the goal nodes
bool goal(node_value n) {
	bool result = false;
	for (goals_const_iter_t g = goals.begin(); g != goals.end(); ++g) {
		if ( (n.i == (*g).i) && (n.j == (*g).j) ) {
			result = true;
			break;
		}
	}
	return result;
}

int main()
{
	zones.push_back(zone(20.0, 20.0, 10.0));
	zones.push_back(zone(40.0, 0.0, 20.0));
	zones.push_back(zone(0.0, 15.0, 8.0));

	// sample obstacle generating function
	for (long j = jmin; j <= jmax; ++j) {
		for (long i = imin; i <= imax; ++i) {
			float x = getX(i,j);
			float y = getY(j);
			obstacles[i][j] = 0;
			for (zones_iter_t pz = zones.begin(); pz != zones.end(); ++pz) {
				zone z = *pz;
				if (z.contains(x, y)) {
					obstacles[i][j] = 1;
					break;
				}
			}
		}
	}

	// single node in a goals' set
	goals.push_back(node_value(imax, jmax));
	
	typedef ExpTr<node_value,cost_value,expand,less<cost_value> > ExpTr_t;
	ExpTr_t tr(node_value(imin,jmin));

	cout << "in exp traverser forward search ...\n";
	tr.set_handler_on_expand_root(OnExpandRoot);
	tr.set_handler_on_select_cursor(OnSelectCursor);
	// lAAAUUUnch it finally!
	ForwardSearch(tr, goal);
	ShowPath(tr);
	
	cout << "number of expansions = " << num_expansions << "\n";
	cout << "...out \n\n";

	cout << "Search is completed.\n\n";

	// draw results - the view is not normalized
	// grid is hexagonal and is sgown in rectangular manner
	cout << "MAP AND PATH - (o-obstacle, *-waypoint)\n\n";
	for (long _j = jmin; _j <= jmax; ++_j) {
		for (long _i = imin; _i <= imax; ++_i) {
			if (ContainsNode(tr, node_value(_i,_j))) {
				cout << "*";
			} else if ( obstacles[_i][_j] == 0 ) {
				cout << " ";
			} else {
				cout << "o";
			}
		}
		cout << '\n';
	}
	cout << "\n----------------END OF MAP----------------\n";

	return 0;
	
}
