/*
Graph Search Library
Copyright(c) 1999-2003 

Actually generic algorithms.
*/

#ifndef SOLVER_H
#define SOLVER_H

#include "const.h"

// Simple forward search
template <class Tr, class Goal>
void ForwardSearch(Tr& n, Goal goal) {
	while( !n.empty() && !goal( n.cursor() ) )
		n.move_forward();
}

// Finds solution not exceeding given value
template <class Tr, class Goal>
void BoundedSearch(Tr &n, Goal goal, const typename Tr::cost_t &bound) {
	while( !n.empty() ) {
		if ( n.cost() > bound )	n.backtrack();
		else if( goal( n.cursor() ) ) return;
		else	n.move_forward();
	}
}

// Finds optimal solution not exceeding given value
template <class Tr, class Goal>
void BoundedOptSearch(Tr &n, Goal goal, const typename Tr::cost_t &bound) {
	if( n.empty() ) return;
	Tr m;
	m.swap(n);
	typename Tr::cost_t new_bound = bound;
	while( !m.empty() ) {
		if( m.cost() > new_bound) m.backtrack();
		else if( goal( m.cursor() ) ) {
			new_bound = m.cost();
			if( n.empty()  || ( m.cost() < n.cost() ) )
				n = m;
			m.backtrack();
		} else m.move_forward();
	}
}

// Branch and bound
template <class Tr, class Goal>
void BranchAndBound(Tr &n, Goal goal)
{
	while ( !n.empty() && !goal( n.cursor() ) )
	    n.move_forward();
	if ( n.empty() ) return;
	Tr m = n;
	n.backtrack();
	while ( !n.empty() )
	{
	    if ( goal( n.cursor() ) && n.cost() < m.cost() )	{ m = n; n.backtrack(); }
	    else if ( n.cost() > m.cost() ) n.backtrack();
	    else n.move_forward();
	}
	n.swap(m);
}

// Iterative Deepening
template <class Tr, class Goal>
void IterativeDeepening(Tr& n, Goal goal)
{
	if( n.empty() ) return;
	Tr m = n;

	typename Tr::cost_t thresh = m.cost();
	typename Tr::cost_t next_thresh = PLUS_INFINITY;

	for(;;)	{
		do {
			if ( m.cost() <= thresh) {
				if ( goal( m.cursor() ) ) { n.swap(m); return; }
				m.move_forward();
				if( m.empty() ) { n.swap(m); return; }
			}
			else { if ( m.cost() < next_thresh )  next_thresh = m.cost(); m.backtrack(); }
		} while( !m.empty() );
		m = n;
		thresh = next_thresh;
		next_thresh = PLUS_INFINITY;
	}
}


// Bidirectional search
template <class Tr1, class Tr2>
void BidirectionalSearch(Tr1& n1, Tr2& n2)
{
	bool first = true;
	while( !n1.empty() & !n2.empty() )
	{
		if(first)	{ if( n2.contains(n1.cursor()) ) return; n1.move_forward(); first = false; }
		else	{ if( n1.contains(n2.cursor()) ) return; n2.move_forward(); first = true; }
	}
}

#endif
