/***************************************************************************
 *   Copyright (C) 2014 by Markus Bader                                    *
 *   markus.bader@tuwien.ac.at                                             *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/


#ifndef TUW_ELLIPSES_DEFAULTS_H
#define TUW_ELLIPSES_DEFAULTS_H

#define TUW_ELLIPSES_DEFAULT_DEBUG false
#define TUW_ELLIPSES_DEFAULT_DISTORTED_INPUT true
#define TUW_ELLIPSES_DEFAULT_EDGE_DETECTION 1
#define TUW_ELLIPSES_DEFAULT_THRESHOLD_EDGE_DETECTION1 150
#define TUW_ELLIPSES_DEFAULT_THRESHOLD_EDGE_DETECTION2 100
#define TUW_ELLIPSES_DEFAULT_KERNEL_SIZE_EDGE_DETECTION 3
#define TUW_ELLIPSES_DEFAULT_EDGE_LINKING 2
#define TUW_ELLIPSES_DEFAULT_THRESHOLD_CONTROUR_MIN_POINTS 20
#define TUW_ELLIPSES_DEFAULT_THRESHOLD_POLYGON 5
#define TUW_ELLIPSES_DEFAULT_FILTER_CONVEX true
#define TUW_ELLIPSES_DEFAULT_ELLIPSE_REDEFINEMENT true
#define TUW_ELLIPSES_DEFAULT_THRESHOLD_ROTATED_RECT_RATIO 30
#define TUW_ELLIPSES_DEFAULT_FILTER_RING true
#define TUW_ELLIPSES_DEFAULT_THRESHOLD_RING_CENTER 0.2
#define TUW_ELLIPSES_DEFAULT_THRESHOLD_MIN_RADIUS 0.0
#define TUW_ELLIPSES_DEFAULT_THRESHOLD_MAX_RADIUS 1.0
#define TUW_ELLIPSES_DEFAULT_THRESHOLD_RING_RATIO 0.1
#define TUW_ELLIPSES_DEFAULT_FILTER_CONTOUR_MEAN true
#define TUW_ELLIPSES_DEFAULT_THRESHOLD_CONTOUR_MEAN 0.1
#define TUW_ELLIPSES_DEFAULT_ESTIMATE_POSE 3
#define TUW_ELLIPSES_DEFAULT_CIRCLE_DIAMETER 0.1

#endif // TUW_ELLIPSES_DEFAULTS_H
