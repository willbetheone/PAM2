// Geometric Tools, LLC
// Copyright (c) 1998-2014
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 5.0.1 (2010/10/01)

#include "Wm5MathematicsPCH.h"
#include "Wm5IntrSegment2Arc2.h"
#include "Wm5IntrLine2Circle2.h"

namespace Wm5
{
//----------------------------------------------------------------------------
template <typename Real>
IntrSegment2Arc2<Real>::IntrSegment2Arc2 (const Segment2<Real>& segment,
    const Arc2<Real>& arc)
    :
    mSegment(&segment),
    mArc(&arc)
{
}
//----------------------------------------------------------------------------
template <typename Real>
const Segment2<Real>& IntrSegment2Arc2<Real>::GetSegment () const
{
    return *mSegment;
}
//----------------------------------------------------------------------------
template <typename Real>
const Arc2<Real>& IntrSegment2Arc2<Real>::GetArc () const
{
    return *mArc;
}
//----------------------------------------------------------------------------
template <typename Real>
bool IntrSegment2Arc2<Real>::Find ()
{
    Real t[2];
    int quantity;
    bool intersects = IntrLine2Circle2<Real>::Find(mSegment->Center,
        mSegment->Direction, mArc->Center, mArc->Radius, quantity, t);

    mQuantity = 0;
    if (intersects)
    {
        // Reduce root count if line-circle intersections are not on segment.
        if (quantity == 1)
        {
            if (Math<Real>::FAbs(t[0]) > mSegment->Extent)
            {
                quantity = 0;
            }
        }
        else
        {
            if (t[1] < -mSegment->Extent || t[0] > mSegment->Extent)
            {
                quantity = 0;
            }
            else
            {
                if (t[1] <= mSegment->Extent)
                {
                    if (t[0] < -mSegment->Extent)
                    {
                        quantity = 1;
                        t[0] = t[1];
                    }
                }
                else
                {
                    quantity = (t[0] >= -mSegment->Extent ? 1 : 0);
                }
            }
        }

        for (int i = 0; i < quantity; ++i)
        {
            Vector2<Real> point = mSegment->Center +
                mSegment->Direction*t[i];

            if (mArc->Contains(point))
            {
                mPoint[mQuantity++] = point;
            }
        }
    }

    mIntersectionType = (mQuantity > 0 ? IT_POINT : IT_EMPTY);
    return mIntersectionType != IT_EMPTY;
}
//----------------------------------------------------------------------------
template <typename Real>
int IntrSegment2Arc2<Real>::GetQuantity () const
{
    return mQuantity;
}
//----------------------------------------------------------------------------
template <typename Real>
const Vector2<Real>& IntrSegment2Arc2<Real>::GetPoint (int i) const
{
    return mPoint[i];
}
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// Explicit instantiation.
//----------------------------------------------------------------------------
template WM5_MATHEMATICS_ITEM
class IntrSegment2Arc2<float>;

template WM5_MATHEMATICS_ITEM
class IntrSegment2Arc2<double>;
//----------------------------------------------------------------------------
}
