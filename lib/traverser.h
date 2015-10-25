/*
Graph Search Library
Copyright(c) 1999-2003 

Traversers.
*/

#ifndef TRAVERSER_H
#define TRAVERSER_H

#include <iostream>
#include <vector>

#include "config.h"

#include "const.h"

#include "treenode.h"

#define __SWAP_VALUES(Type,Tmp,Var1,Var2) Type Tmp = Var1; Var1 = Var2; Var2 = Tmp;

#define __SET_HANDLER(HandlerName) HandlerName##_t set_handler_##HandlerName(HandlerName##_t new_handler = NULL) \
	{ \
	    HandlerName##_t old_handler = HandlerName##_handler; \
	    HandlerName##_handler = new_handler; \
	    return old_handler; \
	}

#define __DECLARE_HANDLER(HandlerName) HandlerName##_t HandlerName##_handler;


using namespace std;

template<class value_type, class cost_type, class exp_type, class Cmp> class LinTr;
template<class value_type, class cost_type, class exp_type, class Cmp> class ExpTr;

template<class value_type, class cost_type, class exp_type, class Cmp>
class RevIter
{
	typedef Node<value_type, cost_type> Node_t;
	typedef MemItem<Node_t> MemNode_t;
	typedef Item<MemNode_t> NodeP_t;

	NodeP_t pCurs;
	void Next() {	if( pCurs ) pCurs = (*pCurs)->pPar; }

	RevIter() : pCurs() {}
	RevIter(const NodeP_t& ApCurs) : pCurs(ApCurs) {}

public:
	friend class LinTr<value_type, cost_type, exp_type, Cmp>;
	friend class ExpTr<value_type, cost_type, exp_type, Cmp>;

	RevIter& operator ++() { Next(); return *this; }
	RevIter operator ++(int ) { RevIter ri = *this; Next(); return ri; }
	bool operator ==(const RevIter& ri) const
	{
		return pCurs.IsNull() && ri.pCurs.IsNull() ? true : pCurs == ri.pCurs;
	}
	bool operator !=(const RevIter& ri) const { return !operator ==(ri); }
	value_type operator *() const
	{
#ifdef DEBUG	
		assert( !pCurs.IsNull() );
#endif		
		return (*pCurs)->Val;
	}
	cost_type operator()() const { assert( !pCurs.IsNull() ); return (*pCurs)->Cost; }
	RevIter& operator =(const RevIter& ri) { if( this != &ri) pCurs = ri.pCurs; return *this;	}
	RevIter(const RevIter& ri) : pCurs(ri.pCurs) {}
};

template <class cmp_t>
class NoCmp
{
public:
	bool operator ()(const cmp_t& c1, const cmp_t& c2) { return 0; }
};

template <class value_t, class cost_t, class cmp_t>
class CostCmp
{
public:
	typedef Node<value_t, cost_t> Node_t;
	typedef MemItem<Node_t> MemNode_t;
	typedef Item<MemNode_t> NodeP_t;

	cmp_t cmp;

	bool operator ()(const NodeP_t& n1, const NodeP_t& n2) { return cmp( (*n1)->Cost, (*n2)->Cost ); }
};

template <class value_type, class cost_type, class exp_type, class Cmp = NoCmp<cost_type> >
class LinTr
{
public:
	typedef value_type value_t;
	typedef cost_type cost_t;
	typedef exp_type exp_t;
	typedef Cmp cmp_t;

	exp_t FExpand;

	typedef list<value_type> ValList_t; // user-defined list
	ValList_t expd;

	typedef list<cost_type> CostList_t; // user-defined list
	CostList_t costs;

	typedef Node<value_t, cost_t> Node_t;
	typedef MemItem<Node_t> MemNode_t;
	typedef Item<MemNode_t> NodeP_t;

	static long NextID;

	// ============ Significant events for LinTr ============
	// root node expanded
	typedef void(*on_expand_root_t)();
	__SET_HANDLER(on_expand_root)
	// no further move possible - empty
	typedef void(*on_become_empty_t)();
	__SET_HANDLER(on_become_empty)
	// successor list received - cursor value, succesor list
	typedef void(*on_receive_childs_t)(const value_t&, const list<value_t>&);
	__SET_HANDLER(on_receive_childs)
	// new cursor selected - cursor value, cursor cost
	typedef void(*on_select_cursor_t)(const value_t&, const cost_t&);
	__SET_HANDLER(on_select_cursor)
	// external cursor deletion - cursor value
	typedef void(*on_delete_cursor_t)(const value_t&);
	__SET_HANDLER(on_delete_cursor)

private:
	__DECLARE_HANDLER(on_expand_root)
	__DECLARE_HANDLER(on_become_empty)
	__DECLARE_HANDLER(on_receive_childs)
	__DECLARE_HANDLER(on_select_cursor)
	__DECLARE_HANDLER(on_delete_cursor)

	long ID, CopyID;
	
	CostCmp<value_t, cost_t, cmp_t> cmp;

	typedef list<NodeP_t> pNds_t;
	typedef typename pNds_t::iterator pNdsIt_t;
	typedef typename pNds_t::const_iterator pNdsConstIt_t;
	typedef typename pNds_t::reverse_iterator pNdsRevIt_t;
	typedef typename pNds_t::const_reverse_iterator pNdsConstRevIt_t;

	typedef vector<NodeP_t> pNdsVec_t;
	typedef typename pNdsVec_t::iterator pNdsVecIt_t;
	typedef typename pNdsVec_t::const_iterator pNdsVecConstIt_t;

	NodeP_t pCurs, pRoot; // tree cursor and root
	pNds_t pNds; // list of ready-to-expand nodes

	bool IsNull() const { return pCurs.IsNull(); }

	void DelCurs()
	{
		if ( IsNull() ) return;
		if( !(*pCurs)->Root() ) ( *(*pCurs)->pPar )->DelCh(pCurs);
		pNds.pop_front();
		if(!pNds.empty()) {
		  pCurs =  pNds.front();
	    } else {
		  pCurs.reset();
		}
		if ( (*pRoot)->Leaf() )
			pRoot.reset();
	}

	void Clear()
	{
		if ( IsNull() ) return;
		(*pRoot)->Prune();
		pCurs.reset();
		pRoot.reset();
		pNds.clear();
	}

	NodeP_t FindNode(const NodeP_t init_node, const value_t val)
	{
		if( (*init_node)->Val == val ) return init_node;
		NodeP_t pN;
		for( pNdsConstIt_t i = (*init_node)->Chs.begin(); i != (*init_node)->Chs.end(); ++i )
		{
			pN = FindNode( *i, val );
			if(pN) return pN;
		}	
		return pN;
	}

	NodeP_t CopyNode(const NodeP_t src, const pNdsVec_t& pSrcVec,  pNdsVec_t& pDestVec)
	{
		NodeP_t  pN( new MemNode_t( Node_t( (*src)->Val, (*src)->Cost ) ) );
		if ( (*src)->Leaf() )
		{
			for(int i = 0; i < pSrcVec.size(); ++i)
				if( pSrcVec[i] == src ) { pDestVec[i] = pN; break; }
		}
		else
		{
			NodeP_t pCh;
			for(pNdsConstIt_t i = (*src)->Chs.begin(); i != (*src)->Chs.end(); ++i)
			{
				(*pN)->Chs.push_back( pCh = CopyNode(*i, pSrcVec, pDestVec) );
				(*pCh)->pPar = pN;
			}	
		}
		return pN;
	}

	void __Clone(const LinTr& tr)
	{
		if ( !IsNull() ) Clear();
		if ( tr.IsNull() ) return;

		pNdsVec_t pSrcVec;
		copy(tr.pNds.begin(), tr.pNds.end(), inserter(pSrcVec, pSrcVec.begin() ) );

		pNdsVec_t pDestVec( pSrcVec.size() );
		pRoot = CopyNode(tr.pRoot, pSrcVec, pDestVec);

		copy(pDestVec.begin(), pDestVec.end(), inserter(pNds, pNds.begin() ) );

		pCurs = pNds.front();
	}

	NodeP_t MakeCopy(const NodeP_t& src)
	{
		NodeP_t  pN( new MemNode_t( Node_t( (*src)->Val, (*src)->Cost ) ) );
		NodeP_t pCh;
		for(pNdsConstIt_t i = (*src)->Chs.begin(); i != (*src)->Chs.end(); ++i)
		{
			(*pN)->Chs.push_back( pCh = MakeCopy(*i) );
			(*pCh)->pPar = pN;
		}

		return pN;
	}

	void __SmartClone(const LinTr& tr)
	{
		// find differences in pNds in reverse order
		pNdsConstRevIt_t i = tr.pNds.rbegin();
		pNdsRevIt_t j = pNds.rbegin(), j1;
		for( ; i != tr.pNds.rend() && j != pNds.rend(); ++i, ++j)
			if( ( *(*i) )->Val != ( *(*j) )->Val )
				break;

		// no differences found
		if( ( i == tr.pNds.rend() ) && ( j == pNds.rend() ) ) return;
		
		// variables
		NodeP_t pN = *i, pM = *j, pCh;

		// prune absent nodes with parents
		if( j != pNds.rend() )
			for( ++(j1 = j); j1 != pNds.rend(); ++j1 )
				( *( ( *(*j1) )->pPar ) )->DelCh(*j1);

		pNdsIt_t n = pNds.begin();
		for( ; n != pNds.end(); ++n )
			if( (*(*n))->Val == (*pM)->Val ) break;
		pNds.erase(pNds.begin(), n);

		// moving up in the tree
		while( (*pN)->Val != (*pM)->Val )
			pN = (*pN)->pPar;
		
		// new nodes creation
		for( pNdsConstIt_t k = (*pN)->Chs.begin(); k != (*pN)->Chs.end(); ++k ) 
		{
			(*pM)->Chs.push_back( pCh = MakeCopy(*k) );
			(*pCh)->pPar = pM;
		}

		// restore nodes order in pNds
		for( ; i !=tr.pNds.rend(); ++i)
			pNds.push_front( FindNode( pM, (*(*i))->Val ) );
			
		pCurs = pNds.front();
		
	}

	void Clone(const LinTr& tr)
	{
		if( CopyID == tr.ID )
		{
			__SmartClone(tr);
		}
		else
		{
			CopyID = tr.ID;
			__Clone(tr);
		}

		on_expand_root_handler = tr.on_expand_root_handler;
		on_become_empty_handler = tr.on_become_empty_handler;
		on_receive_childs_handler = tr.on_receive_childs_handler;
		on_select_cursor_handler = tr.on_select_cursor_handler;
		on_delete_cursor_handler = tr.on_delete_cursor_handler;
		FExpand = tr.FExpand;
	}

	void MoveForward()
	{
		if ( IsNull() )
			return;

#ifdef HANDLERS
		if ( on_expand_root_handler ) {
			if ( (*pRoot)->Leaf() ) {
				on_expand_root_handler();
			}
		}
#endif

		value_t PredVal = (*pCurs)->Root() ? (*pCurs)->Val :
			(*((*pCurs)->pPar))->Val;
		FExpand(expd, costs, (*pCurs)->Val, PredVal, (*pCurs)->Cost);
		(*pCurs)->AddChilds(expd, costs, pCurs);
		expd.clear();
		costs.clear();

#ifdef HANDLERS
		if( on_receive_childs_handler )
		{
			list<value_t> L;
			for(pNdsIt_t i = (*pCurs)->Chs.begin(); i != (*pCurs)->Chs.end(); ++i)
				L.push_back( (*(*i))->Val );
			on_receive_childs_handler( (*pCurs)->Val, L);
		}
#endif

		if ( (*pCurs)->Leaf() ) 
		{
			DelCurs(); 

#ifdef HANDLERS
			if( pNds.empty() )
			{
				if( on_become_empty_handler )
					on_become_empty_handler();
			}
			// new cursor selected
			else	if(on_select_cursor_handler)
				on_select_cursor_handler( (*pCurs)->Val, (*pCurs)->Cost );
#endif
		}
		else
		{
			pNds.pop_front();

			vector<NodeP_t> tmp
#ifdef _MSC_VER
			;
			copy( (*pCurs)->Chs.begin(), (*pCurs)->Chs.end(),
				inserter(tmp, tmp.begin() ) );
#else
			( (*pCurs)->Chs.begin(), (*pCurs)->Chs.end() );
#endif		
			sort( tmp.begin(), tmp.end(), cmp );
			copy( tmp.begin(), tmp.end(),
				inserter( pNds, pNds.begin() ) );

			pCurs = pNds.front();

#ifdef HANDLERS
			// new cursor selected
			if(on_select_cursor_handler)
				on_select_cursor_handler( (*pCurs)->Val, (*pCurs)->Cost );
#endif

		}
	}

	void Backtrack()
	{
#ifdef HANDLERS
		if(on_delete_cursor_handler)
			on_delete_cursor_handler( (*pCurs)->Val );
#endif

		// External cursor deletion
		DelCurs();

#ifdef HANDLERS
		if( pNds.empty() )
		{
			if( on_become_empty_handler )
				on_become_empty_handler();
		}
		// new cursor selected
		else	if(on_select_cursor_handler)
			on_select_cursor_handler( (*pCurs)->Val, (*pCurs)->Cost );
#endif
	}

public:
	void Show(int arg = 0) const { if ( !pRoot.IsNull() ) (*pRoot)->Show(arg); }
	void ShowNodeQueue() const
	{
		for(pNdsConstIt_t i = pNds.begin(); i != pNds.end(); ++i)
			cout << (*i)->Val << ' ';
		cout << '\n';
	}

	RevIter<value_type, cost_type, exp_type,  Cmp> RPathBegin()
		{ return RevIter<value_type, cost_type, exp_type, Cmp>(pCurs); }
	RevIter<value_type, cost_type, exp_type, Cmp> RPathEnd()
		{ return RevIter<value_type, cost_type, exp_type, Cmp>(); }

	RevIter<value_type, cost_type, exp_type, Cmp> RPathBegin() const
		{ return RevIter<value_type, cost_type, exp_type, Cmp>(pCurs); }
	RevIter<value_type, cost_type, exp_type, Cmp> RPathEnd() const
		{ return RevIter<value_type, cost_type, exp_type, Cmp>(); }

	void swap(LinTr& tr)
	{
		if( this != &tr)
		{
			__SWAP_VALUES(on_expand_root_t, t0, on_expand_root_handler, tr.on_expand_root_handler)
			__SWAP_VALUES(on_become_empty_t, t1, on_become_empty_handler, tr.on_become_empty_handler)
			__SWAP_VALUES(on_receive_childs_t, t2, on_receive_childs_handler, tr.on_receive_childs_handler)
			__SWAP_VALUES(on_select_cursor_t, t3, on_select_cursor_handler, tr.on_select_cursor_handler)
			__SWAP_VALUES(on_delete_cursor_t, t4, on_delete_cursor_handler, tr.on_delete_cursor_handler)
			
			__SWAP_VALUES(int, tmpCopyID, CopyID, tr.CopyID)
			__SWAP_VALUES(int, tmpID, ID, tr.ID)
			__SWAP_VALUES(NodeP_t, tmp, pCurs, tr.pCurs)
			__SWAP_VALUES(NodeP_t, tmp2, pRoot, tr.pRoot)
			__SWAP_VALUES(exp_t, tmpFExpand, FExpand, tr.FExpand)
			pNds.swap(tr.pNds);
		}
	}
	void clear() { Clear(); }

	value_type cursor() const { return pCurs.IsNull() ? (int)NULL : (*pCurs)->Val; }
	cost_type cost() const { return pCurs.IsNull() ? PLUS_INFINITY : (*pCurs)->Cost; }
	bool empty() const { return IsNull(); }
	exp_type& getexpfunc() { return FExpand; }
	void move_forward() { MoveForward(); }
	void backtrack() { Backtrack(); }

	// ALL OPERATORS ARE COMPATIBLE WITH PREVIOUS VERSION
	value_type operator *() const { return cursor(); }
	cost_type operator ()() const { return cost(); }
	bool operator !() const { return empty(); } const

	LinTr& operator ++() { move_forward(); return *this; }
	LinTr operator ++(int) { LinTr tr = *this; move_forward(); return tr; }
	LinTr& operator --() { backtrack(); return *this; }
	LinTr operator --(int) { LinTr tr = *this; backtrack(); return tr; }
	// ...

	LinTr& operator =(const LinTr& tr)	{ if( this != &tr) Clone(tr);	return *this;	}

	LinTr() :
		ID(NextID++), CopyID(-1),
		on_expand_root_handler(NULL),
		on_become_empty_handler(NULL),
		on_receive_childs_handler(NULL),
		on_select_cursor_handler(NULL),
		on_delete_cursor_handler(NULL)
	{}
	LinTr(const value_type& AVal, const cost_type& ACost = 0) :
		ID(NextID++), CopyID(-1),
		on_expand_root_handler(NULL),
		on_become_empty_handler(NULL),
		on_receive_childs_handler(NULL),
		on_select_cursor_handler(NULL),
		on_delete_cursor_handler(NULL)
	{
			pNds.push_back( pCurs = pRoot = NodeP_t( new MemNode_t( Node_t(AVal, ACost) ) ) );
	}
	LinTr(const LinTr& tr) : ID(NextID++), CopyID(-1) { Clone(tr); }
	~LinTr() { Clear(); }
};

template <class value_t, class cost_t, class cmp_t>
class PairCostCmp
{
public:
//	typedef Node<value_t, cost_t, exp_t> Node_t;
//	typedef Item<Node_t> NodeP_t;
	typedef Node<value_t, cost_t> Node_t;
	typedef MemItem<Node_t> MemNode_t;
	typedef Item<MemNode_t> NodeP_t;

	typedef pair<const value_t, NodeP_t> pair_t;

	cmp_t cmp;

	bool operator ()(pair_t n1, pair_t n2) { return cmp( (*n1.second)->Cost, (*n2.second)->Cost ); }
};
template <class value_type, class cost_type, class exp_type, class Cmp >
long LinTr<value_type, cost_type, exp_type, Cmp>::NextID = -1;

template <class value_type, class cost_type, class exp_type, class Cmp = NoCmp<cost_type> >
class ExpTr
{
public:
	typedef value_type value_t;
	typedef cost_type cost_t;
	typedef exp_type exp_t;
	typedef Cmp cmp_t;

	exp_t FExpand;

	typedef list<value_type> ValList_t; // user-defined list
	ValList_t expd;

	typedef list<cost_type> CostList_t; // user-defined list
	CostList_t costs;

//	typedef Node<value_t, cost_t> Node_t;
//	typedef Item<Node_t> NodeP_t;
	typedef Node<value_t, cost_t> Node_t;
	typedef MemItem<Node_t> MemNode_t;
	typedef Item<MemNode_t> NodeP_t;

	// ============ Significant events for ExpTr ============
	
	// root node expanded
	typedef void(*on_expand_root_t)();
	__SET_HANDLER(on_expand_root)
	// no further move possible - empty
	typedef void(*on_become_empty_t)();
	__SET_HANDLER(on_become_empty)
	// successor list received - cursor value, succesor list
	typedef void(*on_receive_childs_t)(const value_t&, const list<value_t>&);
	__SET_HANDLER(on_receive_childs)
	// new cursor selected - cursor value, cursor cost
	typedef void(*on_select_cursor_t)(const value_t&, const cost_t&);
	__SET_HANDLER(on_select_cursor)
	// external cursor deletion - cursor value
	typedef void(*on_delete_cursor_t)(const value_t&);
	__SET_HANDLER(on_delete_cursor)
	
	// Significant events for ExpTr (distinct from LinTr)
	// node already exists in clsd list - dup_node value, dup_node cost, dup_node parent
	typedef void(*on_dup_closed_node_t)(const value_t&, const cost_t&, const value_t&);
	__SET_HANDLER(on_dup_closed_node)
	// re-expansion in clsd list - new node value, new node cost
	typedef void(*on_reexp_closed_node_t)(const value_t&, const cost_t&);
	__SET_HANDLER(on_reexp_closed_node)
	// node already exists in opnd list - dup_node value, dup_node cost, dup_node parent
	typedef void(*on_dup_opened_node_t)(const value_t&, const cost_t&, const value_t&);
	__SET_HANDLER(on_dup_opened_node)
	// re-expansion in opnd list - new node value, new node cost
	typedef void(*on_reexp_opened_node_t)(const value_t&, const cost_t&);
	__SET_HANDLER(on_reexp_opened_node)

private:

	__DECLARE_HANDLER(on_expand_root)
	__DECLARE_HANDLER(on_become_empty)
	__DECLARE_HANDLER(on_receive_childs)
	__DECLARE_HANDLER(on_select_cursor)
	__DECLARE_HANDLER(on_delete_cursor)
	__DECLARE_HANDLER(on_dup_closed_node)
	__DECLARE_HANDLER(on_reexp_closed_node)
	__DECLARE_HANDLER(on_dup_opened_node)
	__DECLARE_HANDLER(on_reexp_opened_node)

	Cmp valueCmp;
	PairCostCmp<value_t, cost_t, cmp_t> nodeCmp;

	typedef map<value_t, NodeP_t> pNds_t;
	typedef typename pNds_t::iterator pNdsIt_t;
	typedef typename pNds_t::const_iterator pNdsConstIt_t;

	typedef list<NodeP_t> pList_t;
	typedef typename pList_t::iterator pListIt_t;
	typedef typename pList_t::reverse_iterator pListRevIt_t;
	typedef typename pList_t::const_iterator pListConstIt_t;
	typedef typename pList_t::const_reverse_iterator pListConstRevIt_t;

	pNds_t opnd, clsd;
	NodeP_t pCurs;

	bool IsNull() const { return opnd.empty(); }

	void DelCurs()
	{
		if ( IsNull() ) return;
		opnd.erase((*pCurs)->Val);
		if( !(*pCurs)->Root() )
		{
			( *(*pCurs)->pPar )->DelCh(pCurs);
			for(NodeP_t pN = (*pCurs)->pPar; (*pN)->Leaf(); pN = (*pN)->pPar)
			{
				clsd.erase((*pN)->Val);
				if( (*pN)->Root() )
					break;
			}
		}
		SetCurs();
	}

	void SetCurs()
	{
		if( IsNull() )
			pCurs.reset();
		else
			pCurs = ( *min_element(opnd.begin(), opnd.end(), nodeCmp) ).second;
	}

	void Prune(NodeP_t pN)
	{
		assert(!!pN);

		for( pListIt_t i = (*pN)->Chs.begin(); i != (*pN)->Chs.end(); ++i )
			Prune(*i);
		(*pN)->Chs.clear();
		if( !(*pN)->Root() )
		{
			( *(*pN)->pPar )->DelCh(pN);
			for( NodeP_t pM = (*pN)->pPar; (*pM)->Leaf(); pM = (*pM)->pPar )
				clsd.erase( (*pM)->Val);
		}

		if( clsd.erase((*pN)->Val) == 0 )
			opnd.erase((*pN)->Val);
		if( pN == pCurs )
			SetCurs();
	}

	void Clear() { pCurs.reset(); opnd.clear(); clsd.clear(); }

	void Clone(const ExpTr& tr)
	{
		if ( !IsNull() ) Clear();

		pNdsConstIt_t i, j, k;
		pListIt_t x;
		// make copy of opnd list
		for( i = tr.opnd.begin(); i != tr.opnd.end(); ++i )
			opnd[(*i).first] = NodeP_t( new MemNode_t( Node_t ( *( (*i).second ) ) ) );

		if(opnd.empty()) {
		  pCurs.reset();
		} else {
		  pCurs = ( *min_element(opnd.begin(), opnd.end(), nodeCmp) ).second;
		} 

		// make copy of clsd list
		for(i = tr.clsd.begin(); i != tr.clsd.end(); ++i)
			clsd[(*i).first] = NodeP_t( new MemNode_t( Node_t ( *( (*i).second ) ) ) );

		// initialize parents' pointers in opnd
		for(i = opnd.begin(); i != opnd.end(); ++i)
		{
			if ( !( ( *(*i).second )->Root() ) )
			{
				j = clsd.find( ( *( *(*i).second )->pPar )->Val );
				assert( j != clsd.end() );
				( *(*i).second )->pPar = (*j).second;
			}
		}

		// initialize parents' pointers in clsd 
		for(i = clsd.begin(); i != clsd.end(); ++i)
		{
			if ( !( ( * (*i).second )->Root() ) )
			{
				j = clsd.find( ( *( *(*i).second )->pPar)->Val );
				if ( j == clsd.end() )
				{
					j = opnd.find( ( *( *(*i).second )->pPar)->Val );
					assert( j != opnd.end() );
				}
				( *(*i).second )->pPar = (*j).second;
			}
		}

		NodeP_t pNold, pNnew;
		for( i = tr.clsd.begin(); i != tr.clsd.end(); ++i )
		{
			pNold = (*i).second;
			j = clsd.find( ( *(*i).second )->Val );
			pNnew = (*j).second;
			assert( j != clsd.end() );
			for(x = (*pNold)->Chs.begin(); x != (*pNold)->Chs.end(); ++x)
			{
				k = clsd.find( (*(*x))->Val );
				if ( k == clsd.end() ) k = opnd.find( (*(*x))->Val );
				assert( k != clsd.end() || k != opnd.end() );
				(*pNnew)->Chs.push_back( (*k).second );
			}
		}

		on_expand_root_handler = tr.on_expand_root_handler;
		on_become_empty_handler = tr.on_become_empty_handler;
		on_receive_childs_handler = tr.on_receive_childs_handler;
		on_select_cursor_handler = tr.on_select_cursor_handler;
		on_delete_cursor_handler = tr.on_delete_cursor_handler;
		on_dup_closed_node_handler = tr.on_dup_closed_node_handler;
		on_reexp_closed_node_handler = tr.on_reexp_closed_node_handler;
		on_dup_opened_node_handler = tr.on_dup_opened_node_handler;
		on_reexp_opened_node_handler = tr.on_reexp_opened_node_handler;
		FExpand = tr.FExpand;

	}

	void MoveForward()
	{
		if( IsNull() )
			return;

#ifdef HANDLERS
		if ( on_expand_root_handler ) {
			if ( (*GetpRoot())->Leaf() ) {
				on_expand_root_handler();
			}
		}
#endif

		value_t PredVal = (*pCurs)->Root() ? (*pCurs)->Val :
			(*((*pCurs)->pPar))->Val;
		FExpand(expd, costs, (*pCurs)->Val, PredVal, (*pCurs)->Cost);
		(*pCurs)->AddChilds(expd, costs, pCurs);
		expd.clear();
		costs.clear();

#ifdef HANDLERS
		// successor list received - cursor value, succesor list
		if( on_receive_childs_handler )
		{
			list<value_t> L;
			for(pListIt_t i = (*pCurs)->Chs.begin(); i != (*pCurs)->Chs.end(); ++i)
				L.push_back( (*(*i))->Val );
			on_receive_childs_handler( (*pCurs)->Val, L);
		}
#endif

		pList_t ChsToDel;
		for(pListIt_t i = (*pCurs)->Chs.begin(); i != (*pCurs)->Chs.end(); ++i)
		{
			NodeP_t pN = *i;
			pNdsIt_t n = clsd.find((*pN)->Val);
			if (n != clsd.end())
			{
				NodeP_t pN2 = (*n).second;
#ifdef HANDLERS
				// node already exists in clsd list - dup_node value, dup_node cost, dup_node parent
				if( on_dup_closed_node_handler )
				{
					NodeP_t xpPar = (*pN2)->pPar;
					if( !xpPar.IsNull() )
						on_dup_closed_node_handler( (*pN2)->Val, (*pN2)->Cost, (*xpPar)->Val );
					else
						on_dup_closed_node_handler( (*pN2)->Val, (*pN2)->Cost, NULL );
				}
#endif
				if ( valueCmp( (*pN)->Cost, (*pN2)->Cost) )
				{
#ifdef HANDLERS
					// re-expansion in clsd list - cursor value, node value, node cost
					if( on_reexp_closed_node_handler )
						on_reexp_closed_node_handler( (*pN)->Val, (*pN)->Cost);
#endif
					Prune(pN2);
					opnd[(*pN)->Val] = pN;
				}
				else ChsToDel.push_back(pN);
			}
			else
			{
				n = opnd.find((*pN)->Val);
				if (n != opnd.end())
				{
					NodeP_t pN2 = (*n).second;
#ifdef HANDLERS
					// node already exists in opnd list - dup_node value, dup_node cost, dup_node parent
					if( on_dup_opened_node_handler )
					{
						NodeP_t ypPar = (*pN2)->pPar;
						if( !ypPar.IsNull() )
							on_dup_opened_node_handler( (*pN2)->Val, (*pN2)->Cost, (*ypPar)->Val );
						else
							on_dup_opened_node_handler( (*pN2)->Val, (*pN2)->Cost, NULL );
					}
#endif
					if ( valueCmp((*pN)->Cost, (*pN2)->Cost) )
					{
#ifdef HANDLERS
						// re-expansion in opnd list - cursor value, node value, node cost
						if( on_reexp_opened_node_handler )
							on_reexp_opened_node_handler( (*pN)->Val, (*pN)->Cost);
#endif
						Prune(pN2);
						opnd[(*pN)->Val] = pN;
					}
					else ChsToDel.push_back(pN);
				}
				else	opnd[(*pN)->Val] = pN;
			}
		} // ... for

		for( pListIt_t k = ChsToDel.begin(); k != ChsToDel.end(); ++k )
		{
			pListIt_t j = find((*pCurs)->Chs.begin(), (*pCurs)->Chs.end(), *k);
			assert( j != (*pCurs)->Chs.end() );
			(*pCurs)->Chs.erase(j);
		}

		if ( (*pCurs)->Leaf() )
			DelCurs();
		else 
		{
			opnd.erase((*pCurs)->Val);
			clsd[(*pCurs)->Val] = pCurs;
			SetCurs();
		}

#ifdef HANDLERS
		if( empty() )
		{
			// no further move possible - empty
			if( on_become_empty_handler )
				on_become_empty_handler();
		}
		// new cursor selected
		else	if(on_select_cursor_handler)
			on_select_cursor_handler( (*pCurs)->Val, (*pCurs)->Cost );
#endif

	}

	void Backtrack()
	{
#ifdef HANDLERS
		if(on_delete_cursor_handler)
			on_delete_cursor_handler( (*pCurs)->Val );
#endif

		DelCurs();

#ifdef HANDLERS
		if( empty() )
		{
			// no further move possible - empty
			if( on_become_empty_handler )
				on_become_empty_handler();
		}
		// new cursor selected
		else	if(on_select_cursor_handler)
			on_select_cursor_handler( (*pCurs)->Val, (*pCurs)->Cost );
#endif
	}

	NodeP_t GetpRoot() const
	{
		NodeP_t pRoot;
		if( clsd.empty() ) pRoot = ( *opnd.begin() ).second;
		else {
			pNdsConstIt_t i;
			for(i = clsd.begin(); i != clsd.end(); ++i)
				if ( (*(*i).second)->Root() ) break;
			assert( i != clsd.end() );
			pRoot = (*i).second;
		}
		return pRoot;
	}


public:
	RevIter<value_type, cost_type, exp_type, Cmp> RPathBegin()
		{ return RevIter<value_type, cost_type, exp_type, Cmp>(pCurs); }
	RevIter<value_type, cost_type, exp_type, Cmp> RPathEnd()
		{ return RevIter<value_type, cost_type, exp_type, Cmp>(); }
	RevIter<value_type, cost_type, exp_type, Cmp> RPathBegin(value_type Val)
	{
		pNdsIt_t i = opnd.find(Val);
		if ( i != opnd.end() )
			return RevIter<value_type, cost_type, exp_type, Cmp>( (*i).second );
		else
		{
			i = clsd.find(Val);
			if ( i != clsd.end() )
				return RevIter<value_type, cost_type, exp_type, Cmp>( (*i).second );
			else
				return RevIter<value_type, cost_type, exp_type, Cmp>();
		}
	}

	RevIter<value_type, cost_type, exp_type, Cmp> RPathBegin() const
		{ return RevIter<value_type, cost_type, exp_type, Cmp>(pCurs); }
	RevIter<value_type, cost_type, exp_type, Cmp> RPathEnd() const
		{ return RevIter<value_type, cost_type, exp_type, Cmp>(); }
	RevIter<value_type, cost_type, exp_type, Cmp> RPathBegin(value_type Val) const
	{
		pNdsConstIt_t i = opnd.find(Val);
		if ( i != opnd.end() )
			return RevIter<value_type, cost_type, exp_type, Cmp>( (*i).second );
		else
		{
			i = clsd.find(Val);
			if ( i != clsd.end() )
				return RevIter<value_type, cost_type, exp_type, Cmp>( (*i).second );
			else
				return RevIter<value_type, cost_type, exp_type, Cmp>();
		}
	}

	void swap(ExpTr& tr)
	{
		if( this != &tr)
		{
			__SWAP_VALUES(NodeP_t, tmp, pCurs, tr.pCurs)
			opnd.swap(tr.opnd);
			clsd.swap(tr.clsd);

			__SWAP_VALUES(on_expand_root_t, t0, on_expand_root_handler, tr.on_expand_root_handler)
			__SWAP_VALUES(on_become_empty_t, t1, on_become_empty_handler, tr.on_become_empty_handler)
			__SWAP_VALUES(on_receive_childs_t, t2, on_receive_childs_handler, tr.on_receive_childs_handler)
			__SWAP_VALUES(on_select_cursor_t, t3, on_select_cursor_handler, tr.on_select_cursor_handler)
			__SWAP_VALUES(on_delete_cursor_t, t4, on_delete_cursor_handler, tr.on_delete_cursor_handler)
			__SWAP_VALUES(on_dup_closed_node_t, t5, on_dup_closed_node_handler, tr.on_dup_closed_node_handler)
			__SWAP_VALUES(on_reexp_closed_node_t, t6, on_reexp_closed_node_handler, tr.on_reexp_closed_node_handler)
			__SWAP_VALUES(on_dup_opened_node_t, t7, on_dup_opened_node_handler, tr.on_dup_opened_node_handler)
			__SWAP_VALUES(on_reexp_opened_node_t, t8, on_reexp_opened_node_handler, tr.on_reexp_opened_node_handler)
			__SWAP_VALUES(exp_t, tmpFExpand, FExpand, tr.FExpand)
		}
	}

	void clear() { Clear(); }

	bool operator &(const value_t Val) const { return opnd.find(Val) != opnd.end(); }

	value_type cursor() const { return pCurs.IsNull() ? (int)NULL : (*pCurs)->Val; }
	cost_type cost() const { return pCurs.IsNull() ? PLUS_INFINITY : (*pCurs)->Cost; }
	bool empty() const { return IsNull(); }
	exp_type& getexpfunc() { return FExpand; }
	void move_forward() { MoveForward(); }
	void backtrack() { Backtrack(); }
	bool contains(const value_t Val) const { return opnd.find(Val) != opnd.end(); }

	// ALL OPERATORS ARE COMPATIBLE WITH PREVIOUS VERSION
	value_type operator *() const { return cursor(); }
	cost_type operator ()() const { return cost(); }
	bool operator !() const { return empty; } const

	ExpTr& operator ++() { move_forward(); return *this; }
	ExpTr operator ++(int) { ExpTr tr = *this; move_forward(); return tr; }
	ExpTr& operator --() { backtrack(); return *this; }
	ExpTr operator --(int) { ExpTr tr = *this; backtrack(); return tr; }
	// ...

	ExpTr& operator =(const ExpTr& tr)	{ if( this != &tr) Clone(tr); return *this;	}

	ExpTr() :
		on_expand_root_handler(NULL),
		on_become_empty_handler(NULL),
		on_receive_childs_handler(NULL),
		on_select_cursor_handler(NULL),
		on_delete_cursor_handler(NULL),
		on_dup_closed_node_handler(NULL),
		on_reexp_closed_node_handler(NULL),
		on_dup_opened_node_handler(NULL),
		on_reexp_opened_node_handler(NULL)
	{}
	ExpTr(const value_type& AVal, const cost_type& ACost = 0) :
		on_expand_root_handler(NULL),
		on_become_empty_handler(NULL),
		on_receive_childs_handler(NULL),
		on_select_cursor_handler(NULL),
		on_delete_cursor_handler(NULL),
		on_dup_closed_node_handler(NULL),
		on_reexp_closed_node_handler(NULL),
		on_dup_opened_node_handler(NULL),
		on_reexp_opened_node_handler(NULL)
	{
		opnd[AVal] = pCurs = NodeP_t( new MemNode_t( Node_t(AVal, ACost) ) ) ;
	}
	ExpTr(const ExpTr& tr) {	Clone(tr); }
	~ExpTr() { Clear(); }

};

#endif
