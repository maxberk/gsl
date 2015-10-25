/*
Graph Search Library
Copyright(c) 1999-2003 

Tree node.
*/

#ifndef TREENODE_H
#define TREENODE_H

#ifdef DEBUG

#include <assert.h>
#endif



#include <list>
#include <algorithm>

#include "config.h"
#include "smartptr.h"
#include "item.h"

// TASK-ORIENTED TREE NODE CLASS
template <class value_type, class cost_type>
struct Node
{
	typedef Node<value_type, cost_type> Node_t;
	typedef MemItem<Node_t> MemNode_t;
	typedef Item<MemNode_t> NodeP_t;

	typedef std::list<NodeP_t> ChsList_t; // pointers-to-children list class
	typedef typename ChsList_t::iterator ChsListIt_t; // iterator over it
	typedef typename ChsList_t::const_iterator ChsListConstIt_t; // iterator over it

	typedef std::list<value_type> ValList_t; // user-defined list
	typedef typename ValList_t::iterator ValListIt_t; // iterator over it

	typedef std::list<cost_type> CostList_t; // user-defined list
	typedef typename CostList_t::iterator CostListIt_t; // iterator over it

	value_type Val; // Node contents
	cost_type Cost; // Node cost with high and low propagated bounds
	NodeP_t pPar; // pointer to parent
	ChsList_t Chs; // children's list

	bool Root() { return pPar.IsNull(); } // is Node root
	bool Leaf() { return Chs.empty(); } // is Node leaf

	void DelPar() {
		if ( !Root() && (*pPar)->Leaf() ) { // delete parent Node only if it is leaf
			(*pPar)->DelPar(); // recursion!
			pPar.reset();
		}
	}

	// Ch is pointer to child;
	// Ch is deleted from ChsList, not from memory
	void DelCh(NodeP_t Ch) {
		if ( Leaf() ) return;
		ChsListIt_t i = find(Chs.begin(), Chs.end(), Ch);
		if (i != Chs.end() ) {
			NodeP_t pN = *i;
			Chs.erase(i);

			// if after operation Node becomes leaf
			if ( Leaf() && !Root() ) (*pPar)->DelCh( (*Ch)->pPar ); // Ch->pPar is *this !!!
		}
	}

	void Prune() {
		for(ChsListIt_t i = Chs.begin(); i != Chs.end(); i++) {
			NodeP_t pN = *i;
			(*pN)->Prune();
			pN.reset();
		}
		Chs.clear();
	}
	
	void AddChilds(ValList_t& expd, CostList_t& costs, const NodeP_t& This)	{
#ifdef DEBUG	
		assert( Leaf() );
#endif
		ValListIt_t i = expd.begin();
		CostListIt_t j = costs.begin();
		for(; i != expd.end() && j != costs.end(); i++, j++)
			Chs.push_back( NodeP_t( new MemNode_t( Node_t( *i, *j, This ) )	) );
	}
	
	bool operator ==(const Node& n) const {	return (Val == n.Val); }
	
	Node(value_type AVal, cost_type ACost) : Val(AVal), Cost(ACost) { }
	Node(value_type AVal, cost_type ACost, NodeP_t ApPar) : Val(AVal), Cost(ACost), pPar(ApPar) { }
	
	Node(const Node& n) : Val(n.Val), Cost(n.Cost), pPar(n.pPar) {}
	Node() {}
	~Node() {}

};

#endif
