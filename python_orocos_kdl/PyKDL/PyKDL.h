// Copyright  (C)  2007  Ruben Smits <ruben dot smits at intermodalics dot eu>
//
// Version: 1.0
// Author: Ruben Smits Ruben Smits <ruben dot smits at intermodalics dot eu>
// Author: Zihan Chen <zihan dot chen dot jhu at gmail dot com>
// Author: Matthijs van der Burgh <MatthijsBurgh at outlook dot com>
// Maintainer: Ruben Smits Ruben Smits <ruben dot smits at intermodalics dot eu>
// Maintainer: Matthijs van der Burgh <MatthijsBurgh at outlook dot com>
// URL: http://www.orocos.org/kdl
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

#include <functional>
#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace std {
template <> struct hash<KDL::Vector> {
  size_t operator()(const KDL::Vector &v) const noexcept {
    return hash<double>()(v.x()) ^ hash<double>()(v.y()) ^
           hash<double>()(v.z());
  }
};
template <> struct hash<KDL::Rotation> {
  size_t operator()(const KDL::Rotation &r) const noexcept {
    double qx, qy, qz, qw;
    r.GetQuaternion(qx, qy, qz, qw);
    return hash<double>()(qx) ^ hash<double>()(qy) ^ hash<double>()(qz) ^
           hash<double>()(qw);
  }
};
template <> struct hash<KDL::Frame> {
  size_t operator()(const KDL::Frame &f) const noexcept {
    return hash<KDL::Vector>()(f.p) ^ hash<KDL::Rotation>()(f.M);
  }
};
template <> struct hash<KDL::Twist> {
  size_t operator()(const KDL::Twist &t) const noexcept {
    return hash<KDL::Vector>()(t.vel) ^ hash<KDL::Vector>()(t.rot);
  }
};
template <> struct hash<KDL::Wrench> {
  size_t operator()(const KDL::Wrench &w) const noexcept {
    return hash<KDL::Vector>()(w.force) ^ hash<KDL::Vector>()(w.torque);
  }
};
template <> struct hash<KDL::VectorVel> {
  size_t operator()(const KDL::VectorVel &v) const noexcept {
    return hash<KDL::Vector>()(v.value()) ^ hash<KDL::Vector>()(v.deriv());
  }
};
template <> struct hash<KDL::RotationVel> {
  size_t operator()(const KDL::RotationVel &r) const noexcept {
    return hash<KDL::Rotation>()(r.value()) ^ hash<KDL::Vector>()(r.deriv());
  }
};
template <> struct hash<KDL::TwistVel> {
  size_t operator()(const KDL::TwistVel &t) const noexcept {
    return hash<KDL::Twist>()(t.value()) ^ hash<KDL::Twist>()(t.deriv());
  }
};
template <> struct hash<KDL::FrameVel> {
  size_t operator()(const KDL::FrameVel &f) const noexcept {
    return hash<KDL::Frame>()(f.value()) ^ hash<KDL::Twist>()(f.deriv());
  }
};
} // namespace std

void init_frames(pybind11::module &m);
void init_framevel(pybind11::module &m);
void init_kinfam(pybind11::module &m);
void init_dynamics(pybind11::module &m);
