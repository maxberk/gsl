/*
Graph Search Library
Copyright(c) 1999-2003 

Fast allocator for traverser's nodes
*/
         
# if ! defined (_ITEM_H_)
#define _ITEM_H_

#include <stddef.h>
#include <new>

#ifdef __GNUG__
#pragma interface
#endif

#define DECLARE(Class)          template <class Type> \
                                class Class 
#define INSTANTIATE(Base,Class) template class Class<Base>      ; typedef  Class<Base> Base##Class
#define FREESTORE(Base,Class)   Class<Base>* Class<Base>::FreeStore = 0 
#define INSTANTIATE_ITEM(Base)  INSTANTIATE(Base,MemItem) ; FREESTORE(Base,MemItem)

// #define DEBUG_ITEM_NEW
template <class Type>
class MemItem 
{ public :
  Type          val  ;
  MemItem*         next ;
  static MemItem*  FreeStore ;
  enum {  NewChunk = 1000 } ;
  
#ifdef DEBUG_ITEM_NEW
  int           allocated ; 
#endif

 public :
  typedef Type*                 PType;
  typedef Type&                 RType;
  typedef const Type&   CRType ;
  typedef Type                  TypeOfData ;
  typedef MemItem<Type>    Self ;
  typedef Self                  *PSelf ;
  
  static bool InitStorage(int=NewChunk) ; 
  inline MemItem(const Type& src,PSelf p=0) : next(p) {val=src;} 
  inline MemItem()  {next=0;}
  static inline void* operator new(size_t)
        { 
	     if(!FreeStore) {
		  InitStorage(100);
		}

	  register PSelf p = FreeStore;
#ifdef DEBUG_ITEM_NEW
                if (p->allocated) abort();
                p->allocated = 1 ; 
#endif
          FreeStore = p->next ;
		
          return p ;
          }
          
  static inline void operator delete ( void* p)
    {  
#ifdef DEBUG_ITEM_NEW
                if (!((PSelf)p)->allocated) abort();
                ((PSelf)p)->allocated = 0 ; 
#endif
        ((PSelf)p)->next = FreeStore;
        FreeStore = (PSelf)p;   
        }  
  // typecast operators   
  inline operator PType()                               { return &val;}
  inline operator RType()                               { return val; }
  inline operator CRType() const                { return val; }
  inline operator Type()   const                { return val; }
  
  // smart pointer operator  
  Type * operator ->() throw()                        { return  &val ; }
    
  inline PSelf Next() const                             { return next ; }
  inline void  Concat(PSelf thenext)    { next = thenext ; }
  inline ~MemItem(void)                                    {}
  
  inline static PSelf GetFirst()                { return FreeStore; }
  
};

//#ifndef __GNUG__
template <class Type>
MemItem<Type>* MemItem<Type>::FreeStore = 0 ;
//#endif

template <class Type>
bool MemItem<Type>::InitStorage(int Chunk) 
        { register PSelf p ;
          register PSelf f ;
          f = p = (PSelf) new char[sizeof(Self)*Chunk];

int i;
char * pt = (char*)p;
  for(i=0;i<sizeof(Self)*Chunk;i++) pt[i] = 0;

          if (f)
          {
          for(; p!=&f[Chunk-1];p++)
                {       p->next=p+1;
                        #ifdef DEBUG_ITEM_NEW
                        p->allocated = 0 ; 
                        #endif
                 }
          p->next = FreeStore;
          #ifdef DEBUG_ITEM_NEW
          p->allocated = 0;
          #endif
        FreeStore=f ;
        }
        return (f!=0);          
        }

#endif
