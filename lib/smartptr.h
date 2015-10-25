/*
Portions from BOOST.ORG

(C) Copyright Greg Colvin and Beman Dawes 1998, 1999. Permission to copy,
use, modify, sell and distribute this software is granted provided this
copyright notice appears in all copies. This software is provided "as is"
without express or implied warranty, and with no claim as to its
suitability for any purpose.



Graph Search Library
Copyright(c) 2001 

Smart pointers.
*/

#ifndef SMART_PTR_H
#define SMART_PTR_H

#include <assert.h>
#include <list>
#include <algorithm>

template<typename T> class Item {
  public:
   typedef T element_type;

   explicit Item(T* p =0) : px(p) {
      try { pn = new long(1); }  // fix: prevent leak if new throws
      catch (...) { delete p; throw; } 
   }

   Item(T* p, long* n) : px(p) { ++*(pn = n); } // My stuff

   Item(const Item& r) throw() : px(r.px) { ++*(pn = r.pn); }

   ~Item() { dispose(); }

   Item& operator=(const Item& r) {
      share(r.px,r.pn);
      return *this;
   }

   void reset(T* p=0) {
      if ( px == p ) return;  // fix: self-assignment safe
      if (--*pn == 0) { delete px; }
      else { // allocate new reference counter
        try { pn = new long; }  // fix: prevent leak if new throws
        catch (...) {
          ++*pn;  // undo effect of --*pn above to meet effects guarantee 
          delete p;
          throw;
        } // catch
      } // allocate new reference counter
      *pn = 1;
      px = p;
   } // reset

   bool IsNull() const { return px == 0; } // my stuff 12.07

   operator void*() { return px; } // My stuff
   T& operator*() const throw()  { return *px; }
   T* operator->() const throw() { return px; }
   T* get() const throw()        { return px; }
   // get() is safer!
   operator T*() const throw()   { return px; } 
   bool operator !() const throw() { return *pn == 0; }

   long use_count() const throw(){ return *pn; }
   long* use_count_ptr() const throw() { return pn; }  // My stuff
   bool unique() const throw()   { return *pn == 1; }

   void swap(Item<T>& other) throw()
     { std::swap(px,other.px); std::swap(pn,other.pn); }

private:

   T*     px;     // contained pointer
   long*  pn;     // ptr to reference counter

   void dispose() { if (--*pn == 0) { delete px; delete pn;  px = 0; } }  // px = 0 - my stuff 12.07

   void share(T* rpx, long* rpn) {
      if (pn != rpn) {
         dispose();
         px = rpx;
         ++*(pn = rpn);
      }
   } // share
};  // Item

template<typename T>
  inline bool operator==(const Item<T>& a, const Item<T>& b)
    { return a.get() == b.get(); }

template<typename T>
  inline bool operator!=(const Item<T>& a, const Item<T>& b)
    { return a.get() != b.get(); }

#endif
