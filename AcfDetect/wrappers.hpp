/*
 * wrappers.hpp
 *  for VisualStudio & opencv3.0.0
 *
 *  Created on: Aug 2, 2015
 *		Author: Peng Chao
 *	
 */

#ifndef WRAPPERS_HPP_
#define WRAPPERS_HPP_

#include <stdlib.h>

namespace pc{

// wrapper functions if compiling from C/C++
inline void* wrCalloc( size_t num, size_t size ) { return calloc(num,size); }
inline void* wrMalloc( size_t size ) { return malloc(size); }
inline void wrFree( void * ptr ) { free(ptr); }

// platform independent aligned memory allocation (see also alFree)
inline void* alMalloc( size_t size, int alignment ) {
	const size_t pSize = sizeof(void*), a = alignment-1;
	void *raw = wrMalloc(size + a + pSize);
	void *aligned = (void*) (((size_t) raw + pSize + a) & ~a);
	*(void**) ((size_t) aligned-pSize) = raw;
	return aligned;
}

// platform independent alignned memory de-allocation (see also alMalloc)
inline void alFree(void* aligned) {
	void* raw = *(void**)((char*)aligned-sizeof(void*));
	wrFree(raw);
}

}
#endif /* WRAPPERS_HPP_ */
