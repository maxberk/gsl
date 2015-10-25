/*
Graph Search Library
Copyright(c) 1999-2003 

Path printing procedures using standard i/o.
*/

#ifndef PRNPATH_H
#define PRNPATH_H

#include <iostream>
#include <list>

using namespace std;

template<typename Tr>
void ShowPath(const Tr& tr)
{

	typedef typename Tr::value_t value_t;
	typedef typename Tr::cost_t cost_t;
	typedef typename Tr::exp_t exp_t;
	typedef typename Tr::cmp_t cmp_t;

	typedef RevIter<value_t, cost_t, exp_t,	cmp_t> RevIter_t;

	typedef list<value_t> List_t;
	typedef typename List_t::iterator ListIt_t;
	List_t L;
	for(RevIter_t ri = tr.RPathBegin(); ri != tr.RPathEnd(); ++ri)
		L.push_front(*ri);
	if ( L.empty() )
		cout << "No path found.\n";
	else
	{
		cout << "Path with cost " << tr.cost() << " found: ";
		for(ListIt_t i = L.begin(); i != L.end(); ++i) cout << *i <<  ' '; cout << '\n';
	}
}

template <class Tr1, class Tr2>
void ShowStickedPaths(const Tr1& tr1, const Tr2& tr2)
{
	typedef typename Tr1::value_t value1_t;
	typedef typename Tr1::cost_t cost1_t;
	typedef typename Tr1::exp_t exp1_t;
	typedef typename Tr1::cmp_t cmp1_t;

	typedef typename Tr2::value_t value2_t;
	typedef typename Tr2::cost_t cost2_t;
	typedef typename Tr2::exp_t exp2_t;
	typedef typename Tr2::cmp_t cmp2_t;

	typedef RevIter<value1_t, cost1_t, exp1_t, cmp1_t> RevIter1_t;
	typedef RevIter<value2_t, cost2_t, exp2_t, cmp2_t> RevIter2_t;

	RevIter1_t ri1Begin = tr1.RPathEnd();
	RevIter2_t ri2Begin = tr2.RPathEnd();

	if( tr1 & tr2.cursor() )
	{
		ri1Begin = tr1.RPathBegin( tr2.cursor() );
		ri2Begin = tr2.RPathBegin();
	}
	else if(tr2 & tr1.cursor() )
	{
		ri1Begin = tr1.RPathBegin();
		ri2Begin = tr2.RPathBegin( tr1.cursor() );
	}

	typedef list<value1_t> List_t;
	typedef typename List_t::iterator ListIt_t;
	List_t L;
	for(RevIter1_t ri1 = ri1Begin; ri1 != tr1.RPathEnd(); ++ri1)
		L.push_front(*ri1);
	RevIter2_t ri2 = ri2Begin;
	for(++ri2; ri2 != tr2.RPathEnd(); ++ri2) L.push_back(*ri2);
	if ( L.empty() )
		cout << "No path found.\n";
	else
	{
		cout << "Path with cost " << ri1Begin() + ri2Begin() << " found: ";
		for(ListIt_t i = L.begin(); i != L.end(); ++i) cout << *i <<  ' '; cout << '\n';
	}
}

template<typename Tr, typename node_value>
bool ContainsNode(const Tr &tr, const node_value& n) {
	typedef typename Tr::value_t value_t;
	typedef typename Tr::cost_t cost_t;
	typedef typename Tr::exp_t exp_t;
	typedef typename Tr::cmp_t cmp_t;
	
	typedef RevIter<value_t, cost_t, exp_t,	cmp_t> RevIter_t;
	
	bool result = false;
	for(RevIter_t ri = tr.RPathBegin(); ri != tr.RPathEnd(); ++ri) {
		if ( (*ri) == n ) {
			result = true;
			break;
		}
	}
	return result;
}

#endif
