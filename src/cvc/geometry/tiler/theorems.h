#ifndef __THEOREMS_H__
#define __THEOREMS_H__

#include <ContourTiler/common.h>
#include <ContourTiler/Boundary_slice_chords.h>
#include <ContourTiler/Tiler_workspace.h>

CONTOURTILER_BEGIN_NAMESPACE

bool test_all(const Segment_3& chord, const Tiler_workspace& w);

bool test_theorem2(const Segment_3& chord, const Tiler_workspace& w);

bool test_theorem3(const Segment_3& segment, const Point_3& opposite, Tiler_workspace& w);

bool test_theorem4(const Segment_3& segment, const Point_3& opposite, Tiler_workspace& w);

/// Returns false if the chords fail the test of Theorem 5, described in bajaj96.
bool test_theorem5(const Segment_3& chord0, const Segment_3& chord1);

bool test_theorem5(const Segment_3& chord, const Boundary_slice_chords& bscs);

bool test_theorem5(const Segment_3& chord0, const Segment_3& chord1, const Boundary_slice_chords& bscs);

/// This is a new test not presented in the paper
bool test_theorem8(const Segment_3& segment, const Point_3& opposite, const Tiler_workspace& w);

bool test_theorem9(const Segment_3& segment, const Point_3& opposite, Tiler_workspace& w);

CONTOURTILER_END_NAMESPACE

#endif
