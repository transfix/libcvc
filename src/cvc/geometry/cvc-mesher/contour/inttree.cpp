//----------------------------------------------------------------
//
// intTree.C - interval Tree manipulation
//
// Copyright (c) 1997 Dan Schikore
//
//----------------------------------------------------------------

// $Id: inttree.cpp,v 1.1.1.1 2006/10/11 21:25:51 transfix Exp $

#include <cstdio>
#include <math.h>
#include <cstdlib>
#include <memory.h>
#ifndef WIN32
#include <unistd.h>
#endif

#include <vector>

#include "inttree.h"

#define DEBUG_TREENo

#include "contour_verbose.h"

using namespace std;

//----------------------------------------------------------------
//
// IntTree() - construct a segment tree for the given range of values
//
//----------------------------------------------------------------
IntTree::IntTree(u_int n, float *val)
{
   nseed=0;
   seedsize=0;
   cellid = NULL;
   min = NULL;
   max = NULL;

   if (n==0) {
      nleaf=0;
      vals=NULL;
      minlist = NULL;
      maxlist = NULL;

      return;
   }

   Init(n, val);
}

//----------------------------------------------------------------
//
// Init() - Initialize the segment tree for the given set of values
//
//----------------------------------------------------------------
void
IntTree::Init(u_int n, float *val)
{
   nleaf = n;
   vals = (float *)malloc(sizeof(float)*nleaf);
   memcpy(vals, val, sizeof(float)*nleaf);
   minlist = new CellBucket[nleaf];
   maxlist = new CellBucket[nleaf];
}

//----------------------------------------------------------------
//
// ~IntTree() - free storage for a segment tree
//
//----------------------------------------------------------------
IntTree::~IntTree()
{
if (verbose)
printf("IntTree destructor\n");
   free(vals);
   /* should free inside buckets here */
   delete [] minlist;
   delete [] maxlist;
}

// A seed of a leaf's list with the bound the list is sorted by. mincmp and maxcmp compare that
// bound alone, the comparisons they made reading the tree through the global_tree global, so
// qsort leaves the seeds, tied bounds included, in the order it did then.
struct BoundedSeed {
   float bound;
   u_int seed;
};

int
IntTree::mincmp(const void *p1, const void *p2)
{
   float b1 = ((const BoundedSeed*)p1)->bound;
   float b2 = ((const BoundedSeed*)p2)->bound;

   if (b1 < b2)
      return(-1);
   if (b1 > b2)
      return(1);
   return(0);
}

int
IntTree::maxcmp(const void *p1, const void *p2)
{
   float b1 = ((const BoundedSeed*)p1)->bound;
   float b2 = ((const BoundedSeed*)p2)->bound;

   if (b1 > b2)
      return(-1);
   if (b1 < b2)
      return(1);
   return(0);
}

// sortSeeds() - sort n seeds by min ascending, or with by_max by max descending
void
IntTree::sortSeeds(u_int *seeds, u_int n, int by_max)
{
   vector<BoundedSeed> bounded(n);
   u_int i;

   if (n == 0)
      return;
   for (i=0; i<n; i++) {
      bounded[i].bound = by_max ? seedMax(seeds[i]) : seedMin(seeds[i]);
      bounded[i].seed = seeds[i];
   }
   qsort(&bounded[0], n, sizeof(BoundedSeed), by_max ? maxcmp : mincmp);
   for (i=0; i<n; i++)
      seeds[i] = bounded[i].seed;
}

void
IntTree::Done(void)
{
   int i;

   for (i=0; i<nleaf; i++) {
      sortSeeds(minlist[i].getCells(), maxlist[i].nCells(), 0);
      sortSeeds(maxlist[i].getCells(), maxlist[i].nCells(), 1);
   }
}

//----------------------------------------------------------------
//
// InsertSet() - recursively insert a segment into the tree
//
//----------------------------------------------------------------
void
IntTree::InsertSeg(u_int cellid, float min, float max)
{
   u_int left, right, root;
   u_int n;

   n = addSeed(cellid, min, max);

   left = 0;
   right = nleaf-1;

#ifdef DEBUG_TREE
printf("inserting cell %d (%f %f)\n", cellid, min, max);
#endif

   while (left < right) {
      root = (left + right) >> 1;

#ifdef DEBUG_TREE
printf("comparing with split value %f (node %d)\n", vals[root], root);
#endif

      if (min <= vals[root] && vals[root] <= max) {
         minlist[root].insert(n);
         maxlist[root].insert(n);
         return;
      }

      if (min > vals[root]) {
         left=root+1;
#ifdef DEBUG_TREE
printf("left->root+1\n");
#endif
      }
      else /* max < vals[root] */ {
         right=root-1;
#ifdef DEBUG_TREE
printf("right->root-1\n");
#endif
      }
#ifdef DEBUG_TREE
sleep(1);
#endif
   }

   // left == right
   minlist[left].insert(n);
   maxlist[left].insert(n);
}

void
IntTree::travFun(u_int n, void *data)
{
   IntTree *tree = (IntTree *)data;

   tree->travCB(tree->seedID(n), tree->travData);
}

//----------------------------------------------------------------
//
// Traverse() - Traverse the tree, calling the given function for
//              each stored segment containing the given value
//
//----------------------------------------------------------------
void
IntTree::Traverse(float val, void (*f)(u_int, void *), void *data)
{
   int left, right, root;

   left = 0;
   right = nleaf-1;
   travCB = f;
   travData = data;

   while (left < right) {
      root = (left + right) >> 1;

      if (vals[root] > val) {
         minlist[root].traverseCells(travFun, this);
         right=root-1;
      }
      else {
         maxlist[root].traverseCells(travFun, this);
         left=root+1;
      }
   }
}

//----------------------------------------------------------------
//
// getCells() - traverse the tree, storing the cell id's of all
//              segments containing the given value in a list
//
//----------------------------------------------------------------
u_int
IntTree::getCells(float val, u_int *cells)
{
   int left, right, root;
   u_int ncells;
   int i;

   left = 0;
   right = nleaf-1;

   ncells=0;
   while (left < right) {
      root = (left + right) >> 1;

      if (vals[root] > val) {
         // for all cells in minlist, we know max > val
         // search the minlist for all cells with min < val
         for (i=0; i<minlist[root].nCells(); i++)
            if (seedMin(minlist[root].getCell(i)) < val)
               cells[ncells++]  = seedID(minlist[root].getCell(i));
            else
               break;
         right=root-1;
      }
      else {
         // for all cells in maxlist, we know min < val
         // search the maxlist for all cells with max > val
         for (i=0; i<maxlist[root].nCells(); i++)
            if (seedMax(maxlist[root].getCell(i)) > val)
               cells[ncells++]  = seedID(maxlist[root].getCell(i));
            else
               break;
         left=root+1;
      }
   }

   // map the seed numbers to cell id's
//   for (i=0; i<ncells; i++)
//      cells[i] = seedID(cells[i]);

   return(ncells);
}

//----------------------------------------------------------------
//
// Dump() - dump the tree
//
//----------------------------------------------------------------
void
IntTree::Dump(void)
{
   int i, j;

   for (i=0; i<nleaf; i++) {
      printf("%d: value %f\n", i, vals[i]);
      minlist[i].dump("   MIN:");
      maxlist[i].dump("   MAX:");
      printf("seeds: ");
      for (j=0; j<minlist[i].nCells(); j++) {
         printf("(%d %f %f)", seedID(minlist[i].getCell(j)),
                              seedMin(minlist[i].getCell(j)),
                              seedMax(minlist[i].getCell(j)));
      }
      printf("\n");
   }
}

//----------------------------------------------------------------
//
// Info() - print some stats about the tree
//
//----------------------------------------------------------------
void
IntTree::Info(void)
{
   int i, total, max;

   printf("______INTERVAL TREE STATS_____\n");
   printf("%d total segments\n", nseed);

   printf("%d values in segment tree (%d buckets)\n", nleaf, nleaf*2);

   total = max = 0;
   for (i=0; i<nleaf; i++) {
      total += minlist[i].nCells();
      total += maxlist[i].nCells();
      if (minlist[i].nCells() > max)
          max = minlist[i].nCells();
      if (maxlist[i].nCells() > max)
          max = maxlist[i].nCells();
   }

   printf("total labels in tree: %d\n", total);
   printf("maximum labels in one list: %d\n", max);
   printf("______INTERVAL TREE STATS_____\n");
}
