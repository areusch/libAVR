/* @file external-const.h
 * Common implementation of "external const-ness" for util files.
 * Copyright (C) 2010 Andrew Reusch <areusch@gmail.com>
 *
 * This file is a utility file for header files wishing to implement "external
 * const-ness" in their data structures.
 *
 * What is external const-ness? Interfaces are not supposed to specify
 * implementation details; the standard C way to do this is to provide
 * API clients with pointers to opaque data structures (i.e. a pointer to a
 * struct that has been forward declared in the .h file), and then
 * defining the contents of that data structure in the C file. This typically
 * manifests as follows:
 *
 * linked_list.h:
 *   typedef struct LinkedList LinkedList;
 *
 *   LinkedList* create_linked_list();
 *
 * linked-list.c:
 *   struct LinkedList {
 *       LinkNode* head;
 *       LinkNode* tail;
 *       int size;
 *       // ...
 *   }
 *
 *   LinkedList* create_linked_list() {
 *      LinkedList* new_list = (LinkedList*) malloc(sizeof LinkedList);
 *      // initialize list...
 *      return new_list;
 *   }
 *
 * This way, API clients cannot modify internal members of data structures
 * without writing egregiously-obvious C code (i.e. mucking with bits, etc).
 *
 * Unfortunately, we are banned from using malloc. However, there is another
 * way! The keyword const can be used to indicate that a data member may not
 * be changed. To leverage this, we need a way to indicate that a data member
 * is const to API clients, and mutable to implementations.
 *
 * This method checks to see if a "define guard" is #defined for the C
 * implementation file. If so, it will #define EXTERNAL_CONST to nothing; if
 * not, it will #define EXTERNAL_CONST to const.
 *
 * Then, simply define your data structure in the .h file using EXTERNAL_CONST
 * on all members you wish to make const from the POV of your clients:
 *
 * linked_list.h:
 *   struct LinkedList {
 *       LinkNode* EXTERNAL_CONST head;
 *       LinkNode* EXTERNAL_CONST tail;
 *       EXTERNAL_CONST int size;
 *   } LinkedList;
 *
 * Contributions:
 */


/* Usage:
 * In any file considered to contain implementations of your API, place the
 * following before you #include the interface-defining header file:
 *
 * #define <<MODULE_NAME>> 0xEC
 *
 * In any file considered to define interface-level data structures, include
 * the following code after all #includes have been made:
 *
 * #define EXTERNAL_CONST_INTERFACE <<MODULE_NAME>>
 * #include "util/external-const.h"
 *
 * <<MODULE_NAME>> must be a unique name amongst the code base.
 */

#if EXTERNAL_CONST_INTERFACE == 0xEC
#undef EXTERNAL_CONST
#define EXTERNAL_CONST
#undef EXTERNAL_CONST_IMPLEMENTATION
#else
#undef EXTERNAL_CONST
#define EXTERNAL_CONST const
#undef EXTERNAL_CONST_IMPLEMENTATION
#define EXTERNAL_CONST_IMPLEMENTATION
#endif

#undef EXTERNAL_CONST_INTERFACE
