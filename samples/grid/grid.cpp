/*
Graph Search Library
Copyright(c) 1999-2003 

Grid sample.
The problem is to find a path between two points on a graph. The graph has a regular structure as follows

-------
|  /  /
| /| /|
|/ |/ |
------- ...
|  /  /
| /| /|
|/ |/ |
-------

This grid contains three types of edges - vertical, horizontal and diagonal.
The cost values are assigned to them. The grid has 9 verticals and 7 horizontals (so, totally, 63
nodes). The path should connect the left low node with high upper.
Each node is enumerated by two digit (or one digit number, if the first one is 0). The first digit
is the vertical number and the second is horizontal number. To store costs data which is represented
by integer values the STL map<int, map<int,int> > strcuture is used. For example, c[4][15]=23 means, that
a cost of diagonal edge between (0,4) node and (1,5) node is 23.


*/

#if defined(_MSC_VER)
#ifdef _DEBUG
#pragma warning( disable : 4786 )
#endif
#endif

#include <functional>
#include <map>
 
#include "config.h" 
#include "traverser.h"
#include "solver.h"
#include "prnpath.h"

map<int, map<int,int,less<int> >, less<int> > C;

class expand
{
public:
	void operator()(list<int> &e, list<int>& ce, int n, int, int c)
	{
		for( map<int,int,less<int> >::iterator i = C[n].begin(); i != C[n].end(); ++i) {
			e.push_back( (*i).first );
			ce.push_back( c + (*i).second );
		}
	}
};

class bidir_expand
{
public:
	void operator()(list<int> &e, list<int>& ce, int n, int, int c)
	{
		for( map<int, map<int,int,less<int> >, less<int> >::iterator i = C.begin(); i != C.end(); ++i)
		{
			map<int,int,less<int> > m = (*i).second;

			map<int,int,less<int> >::iterator j = m.find(n);
			if ( j != m.end() )	{
				e.push_back( (*i).first);
				ce.push_back( c + (*j).second );
			}
		}		
	}
};

// 
int start = 0; // start node
int final = 86; // final node
bool goal(int n) { return n == 86; } // goal node predicate
int thresh = 168; // first cost thresh
int next_thresh = 158; // second cost thresh
int next_thresh2 = 148; // third cost thresh
int opt_value = 136; // optimal cost value
int less_than_opt_value = 135; // cost value less than optimal one

// Callback functions, that arer used to display search process steps
// Similar for both traversers types
void OnBecomeEmpty() { cout << "Search space exhausted.\n"; }
void OnReceiveChilds(const int& i, const list<int>& L)
{
	cout << "After node " << i << " expanding one gets";
	for(list<int>::const_iterator n =  L.begin(); n != L.end(); ++n)
		cout << " " << *n;
	cout << "\n";
}
void OnSelectCursor(const int& i, const int& cost) { cout << "Node " << i << " with cost " << cost << " selected.\n"; }
void OnDeleteCursor(const int& i) { cout << "Backtrack from node " << i << '\n'; }
// ...

// For ExpTr only
void OnDupClosedNode(const int& i, const int& cost, const int& prev)
	{ cout << "Node " << i << " with cost " << cost << " and parent " << prev << " already closed.\n"; }
void OnReexpClosedNode(const int& i, const int& cost)
	{ cout << "Node " << i << " with cost " << " re-expands similar in closed.\n"; }
void OnDupOpenedNode(const int& i, const int& cost, const int& prev)
	{ cout << "Node " << i << " with cost " << cost << " and parent " << prev << " already opened.\n"; }
void OnReexpOpenedNode(const int& i, const int& cost)
	{ cout << "Node " << i << " with cost " << cost << " re-expands similar in opened.\n"; }
// ...

int main()
{

// grid map - nodes are enumerated in a following order - first digit - grid column number,
// second digit - row number ("0" is "0,0", "1" is "0,1", etc.)

// vertical edges
	C[0][1] = 11; C[1][2] = 10; C[2][3] = 8; C[3][4] = 7; C[4][5] = 9; C[5][6] = 10;
	C[10][11] = 9; C[11][12] = 8; C[12][13] = 7; C[13][14] = 6; C[14][15] = 8; C[15][16] = 12;
	C[20][21] = 10; C[21][22] = 10; C[22][23] = 8; C[23][24] = 8; C[24][25] = 9; C[25][26] = 13;
	C[30][31] = 12; C[31][32] = 11; C[32][33] = 9; C[33][34] = 7; C[34][35] = 10; C[35][36] = 13;
	C[40][41] = 13; C[41][42] = 10; C[42][43] = 9; C[43][44] = 8; C[44][45] = 11; C[45][46] = 14;
	C[50][51] = 14; C[51][52] = 11; C[52][53] = 10; C[53][54] = 10; C[54][55] = 12; C[55][56] = 15;
	C[60][61] = 14; C[61][62] = 12; C[62][63] = 11; C[63][64] = 11; C[64][65] = 13; C[65][66] = 14;
	C[70][71] = 12; C[71][72] = 10; C[72][73] = 8; C[73][74] = 10; C[74][75] = 12; C[75][76] = 13;
	C[80][81] = 10; C[81][82] = 9; C[82][83] = 7; C[83][84] = 9; C[84][85] = 12; C[85][86] = 14;

// horizontal edges
	C[0][10] = 12; C[10][20] = 11; C[20][30] = 10; C[30][40] = 9; C[40][50] = 13; C[50][60] = 14; C[60][70] = 17; C[70][80] = 20;
	C[1][11] = 11; C[11][21] = 9; C[21][31] = 8; C[31][41] = 7; C[41][51] = 9; C[51][61] = 13; C[61][71] = 14; C[71][81] = 19;
	C[2][12] = 10; C[12][22] = 8; C[22][32] = 8; C[32][42] = 8; C[42][52] = 10; C[52][62] = 12; C[62][72] = 13; C[72][82] = 15;
	C[3][13] = 12; C[13][23] = 13; C[23][33] = 12; C[33][43] = 10; C[43][53] = 11; C[53][63] = 13; C[63][73] = 15; C[73][83] = 20;
	C[4][14] = 14; C[14][24] = 13; C[24][34] = 12; C[34][44] = 10; C[44][54] = 9; C[54][64] = 8; C[64][74] = 11; C[74][84] = 15;
	C[5][15] = 17; C[15][25] = 14; C[25][35] = 13; C[35][45] = 10; C[45][55] = 11; C[55][65] = 13; C[65][75] = 14; C[75][85] = 17;
	C[6][16] = 20; C[16][26] = 18; C[26][36] = 16; C[36][46] = 15; C[46][56] = 14; C[56][66] = 12; C[66][76] = 15; C[76][86] = 17;

// diagonal edges
	C[5][16] = 28;
	C[4][15] = 23; C[15][26] = 20;
	C[3][14] = 18; C[14][25] = 21; C[25][36] = 27;
	C[2][13] = 19; C[13][24] = 20; C[24][35] = 21; C[35][46] = 25;
	C[1][12] = 18; C[12][23] = 17; C[23][34] = 21; C[34][45] = 19; C[45][56] = 27;
	C[0][11] = 20; C[11][22] = 18; C[22][33] = 18; C[33][44] = 19; C[44][55] = 22; C[55][66] = 26;
	C[10][21] = 20; C[21][32] = 19; C[32][43] = 18; C[43][54] = 20; C[54][65] = 22; C[65][76] = 28;
	C[20][31] = 20; C[31][42] = 18; C[42][53] = 19; C[53][64] = 21; C[64][75] = 22; C[75][86] = 29;	
	C[30][41] = 20; C[41][52] = 19; C[52][63] = 22; C[63][74] = 23; C[74][85] = 25;
	C[40][51] = 23; C[51][62] = 24; C[62][73] = 22; C[73][84] = 24;
	C[50][61] = 24; C[61][72] = 23; C[72][83] = 24;
	C[60][71] = 26; C[71][82] = 26;
	C[70][81] = 29;

	typedef LinTr<int,int,expand,greater<int> > LinTr_t; // linear ttraverser
	typedef ExpTr<int,int,expand,less<int> > ExpTr_t; // exponential traverser
	typedef ExpTr<int,int,bidir_expand,less<int> > RevExpTr_t; // second traverser for bi-directional search

	LinTr_t i;
	ExpTr_t j;
	RevExpTr_t k;

	// LINEAR TRAVERSER IN WORK
	/**/
	cout << "in lin traverser forward search ...\n";
	ForwardSearch(i = LinTr_t(start), goal);
	ShowPath(i);
	cout << "...out \n\n";
	/**/

	/*
	cout << "in lin traverser search bounded by " << thresh << "...\n";
	BoundedSearch(i = LinTr_t(start), goal, thresh);
	ShowPath(i);
	cout << "...out \n\n";

	cout << "in lin traverser search bounded by " << next_thersh << "... continue previous one ... \n";
	BoundedSearch(i, goal, next_thresh);
	ShowPath(i);
	cout << "...out \n\n";

	cout << "in lin traverser search bounded by " << next_thresh2 << "... continue previous one ... \n";
	BoundedSearch(i, goal, next_thresh2);
	ShowPath(i);
	cout << "...out \n\n";
	*/

	/*
	cout << "in lin traverser search bounded by " << less_than_opt_value << "...\n";
	BoundedSearch(i = LinTr_t(start), goal, less_than_opt_value);
	ShowPath(i);
	cout << "...out \n\n";
	*/

	/*
	cout << "in lin traverser optimal search bounded by " << thresh << " from initial state ...\n";
	i = LinTr_t(start);
    	BoundedOptSearch(i, goal, thresh);
	ShowPath(i);
	cout << "...out \n\n";
	*/

	/*
	cout << "in lin traverser optimal search bounded by " << opt_value << " from initial state ...\n";
    	BoundedOptSearch(i = LinTr_t(start), goal, opt_value);
	ShowPath(i);
	cout << "...out \n\n";
	*/

	/*
        cout << "in lin traverser branch and bound ...\n";
	BranchAndBound(i = LinTr_t(start), goal);
	ShowPath(i);
	cout << "...out \n\n";
	*/

// EXPONENTIAL TRAVERSER IN WORK
	/**/
	cout << "in exp traverser forward search ...\n";
	ForwardSearch(j = ExpTr_t(start), goal);
	ShowPath(j);
	cout << "...out \n\n";
	/**/

	/*
	cout << "in exp traverser search bounded by " << thersh << "...\n";
	BoundedSearch(j = ExpTr_t(start), goal, thresh);
	ShowPath(j);
	cout << "...out \n\n";
	*/

	/*
	cout << "in exp traverser search bounded by " << next_thersh << "... continue previous one...";
	BoundedSearch(j, goal, next_thresh);
	ShowPath(j);
	cout << "...out \n\n";
	*/

	/*
        cout << "in exp traverser optimal search from initial state bounded by " << thresh << "...\n";
	BoundedOptSearch(j = ExpTr_t(start), goal, thresh);
	ShowPath(j);
	cout << "...out \n\n";
	*/

	/*
        cout << "in exp traverser branch and bound ...\n";
	BranchAndBound(j = ExpTr_t(start), goal);
	ShowPath(j);
	cout << "...out \n\n";
	*/

	/*
	cout << "in exp traverser iterative deepening ...\n";
	IterativeDeepening(j = ExpTr_t(start), goal);
	ShowPath(j);
	cout << "...out \n\n";
	*/

	/*
	cout << "in exp traverser bi-directional search ...\n";
	BidirectionalSearch( j = ExpTr_t(start), k = RevExpTr_t(final) );
	ShowStickedPaths(j,k);
	cout << "...out \n\n";
	*/

	cout << "All searches completed.\n";

	return 0;
}
