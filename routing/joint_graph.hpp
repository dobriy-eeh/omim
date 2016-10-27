#pragma once

#include "routing/road_graph.hpp"

#include "indexer/feature.hpp"
#include "indexer/index.hpp"

#include "coding/reader.hpp"

#include "geometry/point2d.hpp"

#include "std/cstdint.hpp"


namespace routing
{
class SegPoint final
{
public:
  SegPoint() = default;

  SegPoint(const SegPoint &) = default;

  SegPoint(uint32_t featureId, uint32_t segId)
    : m_featureId(featureId)
    , m_segId(segId)
  {}

  uint32_t GetFeatureId() const { return m_featureId; }

  uint32_t GetSegId() const { return m_segId; }

  bool operator < (SegPoint const & point) const
  {
    if ( m_featureId != point.m_featureId )
      return m_featureId < point.m_featureId;

    return m_segId < point.m_segId;
  }

  bool operator == (SegPoint const & point) const
  {
    return m_featureId == point.m_featureId && m_segId == point.m_segId;
  }

  bool operator != (SegPoint const & point) const
  {
    return !(*this == point);
  }

  uint64_t HashCode() const
  {
    return (static_cast<uint64_t>(m_featureId) << 32) + static_cast<uint64_t>(m_segId);
  }

  friend string DebugPrint(SegPoint const & r);

  template <class TSink>
  void Serialize(TSink & sink) const
  {
    WriteToSink(sink, m_featureId);
    WriteToSink(sink, m_segId);
  }

  template <class TSource>
  void Deserialize(TSource & src)
  {
    m_featureId = ReadPrimitiveFromSource<decltype(m_featureId)>(src);
    m_segId = ReadPrimitiveFromSource<decltype(m_segId)>(src);
  }

private:
  uint32_t m_featureId;
  uint32_t m_segId;
};

class Joint final
{
public:
  void AddPoint(SegPoint const & point)
  {
    m_points.push_back(point);
  }

  size_t GetSize() const { return m_points.size(); }

  SegPoint const & GetPoint(size_t i) const { return m_points[i]; }

  bool operator == (Joint const & joint) const
  {
    if ( m_points.size() != joint.m_points.size())
      return false;

    for (size_t i = 0; i < m_points.size(); ++i)
    {
      if ( m_points[i] != (joint.m_points)[i] )
        return false;
    }
    return true;
  }

  bool operator != (Joint const & joint) const
  {
    return !(*this == joint);
  }

  template <class TSink>
  void Serialize(TSink & sink) const
  {
    WriteToSink(sink, static_cast<uint32_t>(m_points.size()));
    for ( auto const & point: m_points )
    {
      point.Serialize(sink);
    }
  }

  template <class TSource>
  void Deserialize(TSource & src)
  {
    size_t const pointsSize = static_cast<size_t>(ReadPrimitiveFromSource<uint32_t>(src));
    m_points.insert(m_points.end(), pointsSize, SegPoint());
    for (size_t i = 0; i < pointsSize; ++i)
    {
      m_points[i].Deserialize(src);
    }
  }

private:
  vector<SegPoint> m_points;
};

class SegEdge
{
public:
  SegEdge(SegPoint const & target, double weight) : target(target), weight(weight) {}

  inline SegPoint const & GetTarget() const { return target; }

  inline double GetWeight() const { return weight; }

private:
  SegPoint const target;
  double const weight;
};

class JointGraph final
{
public:
  using TVertexType = SegPoint;
  using TEdgeType = SegEdge;

  explicit JointGraph(Index const & index);

  void GetOutgoingEdgesList(SegPoint const & vertex, vector<SegEdge> & edges) const;
  void GetIngoingEdgesList(SegPoint const & vertex, vector<SegEdge> & edges) const;
  double HeuristicCostEstimate(SegPoint const & from, SegPoint const & to) const;

  vector<Junction> ConvertToGeometry(vector<SegPoint> const & vertexes) const;

  template <class TSource>
  void Deserialize(TSource & src)
  {
    size_t const jointsNumber = static_cast<size_t>(ReadPrimitiveFromSource<uint32_t>(src));

    for (size_t i = 0; i < jointsNumber; ++i)
    {
      shared_ptr<Joint> joint = make_shared<Joint>();
      joint->Deserialize(src);
      for (size_t j = 0; j < joint->GetSize(); ++j)
      {
        m_joints[joint->GetPoint(j).HashCode()] = joint;
      }
    }
  }

private:
  void AddAdjacentVertexes(SegPoint const & vertex, vector<SegEdge> & edges) const;
  FeatureType const & GetFeature(uint32_t featureId) const;
  FeatureType const & LoadFeature(uint32_t featureId) const;
  m2::PointD const & GetPoint(SegPoint const & vertex) const;
  SegPoint ResolveVertex(uint32_t featureId, uint32_t segId) const;

  Index const & m_index;
  // @TODO |m_testMwmId| is added for writing prototype. It should be removed. MwmId from |m_mwmLocks|
  // should be used instead.
  MwmSet::MwmId const m_testMwmId;
  map<uint64_t,shared_ptr<Joint>> m_joints;
  mutable map<uint32_t,FeatureType> m_features;
};
}  // namespace routing
