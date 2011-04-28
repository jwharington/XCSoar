/*
 * Copyright (C) 2011 Max Kellermann <max@duempel.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * - Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * - Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the
 * distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * FOUNDATION OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef XCSOAR_SLICE_ALLOCATOR_HPP
#define XCSOAR_SLICE_ALLOCATOR_HPP

#include <cstddef>
#include <assert.h>

/**
 * An optimized allocator for many objects of the same size.  It is
 * extremely efficient at allocating single objects, but never frees
 * up memory to the system heap until it is destructed completely.
 * Released slots will be reused, though.
 *
 * Limitation: this allocator refuses to allocate more than one
 * contiguous object.
 *
 * @param T the type that is wrapped by this allocator
 * @param size the number of objects for each area
 */
template<typename T, unsigned size>
class SliceAllocator {
  typedef std::size_t size_type;

  /**
   * One allocated item in the heap.  When it is on the "available"
   * list, the "next" attribute points to the next available slot.
   * When it is in use, the whole object is casted statically to "T".
   */
  struct Item {
    Item *next;

    char padding[sizeof(T) - sizeof(void *)];
  };

  /**
   * One area holds a certain number of slots.  When all areas are
   * full, a new one is allocated, but empty areas are never freed.
   */
  struct Area {
    Area *next;

    Item *available;

    Item items[size];

#ifndef NDEBUG
    unsigned num_available;
#endif

    Area()
      :next(NULL), available(&items[0])
#ifndef NDEBUG
      , num_available(size)
#endif
    {
      for (unsigned i = 0; i < size - 1; ++i)
        items[i].next = &items[i + 1];
      items[size - 1].next = NULL;
    }

    ~Area() {
      assert(num_available == size);
    }

    Item *allocate() {
      if (available == NULL)
        return NULL;

      Item *i = available;
      available = available->next;

#ifndef NDEBUG
      assert(num_available > 0);
      --num_available;
#endif

      return i;
    }

    bool deallocate(Item *i) {
      if (i < &items[0] || i >= &items[size])
        return false;

      i->next = available;
      available = i;

#ifndef NDEBUG
      ++num_available;
#endif

      return true;
    }
  };

protected:
  /**
   * A linked list of areas.
   */
  Area *head;

public:
  SliceAllocator():head(NULL) {}
  SliceAllocator(const SliceAllocator &other):head(NULL) {}

  ~SliceAllocator() {
    while (head != NULL) {
      Area *area = head;
      head = head->next;
      delete area;
    }
  }

  T *allocate(const size_type n) {
    assert(n == 1);

    /* try to allocate in one of the existing areas */

    Area *area;
    for (area = head; area != NULL; area = area->next) {
      Item *i = area->allocate();
      if (i != NULL)
        return static_cast<T *>(static_cast<void *>(i));
    }

    /* no room, create a new Area and insert it into the linked list */

    area = new Area();
    if (area == NULL)
      /* out of memory */
      return NULL;

    area->next = head;
    head = area;

    /* allocate from the new area */

    Item *i = area->allocate();
    return static_cast<T *>(static_cast<void *>(i));
  }

  void deallocate(T *t, const size_type n) {
    assert(n == 1);

    Item *i = static_cast<Item *>(static_cast<void *>(t));

    for (Area *area = head;; area = area->next) {
      assert(area != NULL);

      if (area->deallocate(i))
        return;
    }

    /* unreachable */
  }

  void construct(T *t, const T& val) {
    new ((void *)t) T(val);
  }

  void destroy(T *t) {
    t->T::~T();
  }
};

#endif
