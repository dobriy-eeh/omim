#include "joint_graph.hpp"

#include "geometry/mercator.hpp"

#include "base/assert.hpp"

namespace
{
inline double CalcDistance(m2::PointD const & from, m2::PointD const & to)
{
  double const distance = MercatorBounds::DistanceOnEarth(from, to);
  return distance;
}
} // namespace

namespace routing
{
JointGraph::JointGraph(Index const & index)
  : m_index(index)
  , m_testMwmId(m_index.GetMwmIdByCountryFile(platform::CountryFile("Russia_Moscow")))
{}

void JointGraph::GetOutgoingEdgesList(SegPoint const & vertexFrom, vector<SegEdge> & edges) const
{
  FeatureType const & feature = GetFeature(vertexFrom.GetFeatureId());
  m2::PointD const & point = feature.GetPoint(vertexFrom.GetSegId());

  if (vertexFrom.GetSegId() > 0)
  {
    uint32_t const adjacentId = vertexFrom.GetSegId() - 1;
    double const distance = CalcDistance(point, feature.GetPoint(adjacentId));
    edges.push_back(SegEdge(SegPoint(vertexFrom.GetFeatureId(),adjacentId), distance));
  }

  if (vertexFrom.GetSegId() < feature.GetPointsCount() - 1)
  {
    uint32_t const adjacentId = vertexFrom.GetSegId() + 1;
    double const distance = CalcDistance(point, feature.GetPoint(adjacentId));
    edges.push_back(SegEdge(SegPoint(vertexFrom.GetFeatureId(),adjacentId), distance));
  }

  auto it = m_joints.find(vertexFrom.HashCode());
  if ( it != m_joints.end())
  {
    Joint const & joint = *it->second;
    for (size_t i = 0; i < joint.GetSize(); ++i)
    {
      SegPoint const & vertex = joint.GetPoint(i);
      if ( vertex != vertexFrom )
      {
        edges.push_back(SegEdge(vertex, HeuristicCostEstimate(vertexFrom, vertex)));
      }
    }
  }
}

void JointGraph::GetIngoingEdgesList(SegPoint const & vertex, vector<SegEdge> & edges) const
{
  GetOutgoingEdgesList(vertex,edges);
}

double JointGraph::HeuristicCostEstimate(SegPoint const & vertexFrom, SegPoint const & vertexTo) const
{
  return CalcDistance(GetPoint(vertexFrom), GetPoint(vertexTo));
}

FeatureType const & JointGraph::GetFeature(uint32_t featureId) const
{
  auto it = m_features.find(featureId);
  if ( it != m_features.end())
    return it->second;

  return LoadFeature(featureId);
}

FeatureType const & JointGraph::LoadFeature(uint32_t featureId) const
{
  Index::FeaturesLoaderGuard guard(m_index, m_testMwmId);
  FeatureType & feature = m_features[featureId];
  bool const isFound = guard.GetFeatureByIndex(featureId,feature);
  if (isFound)
    feature.ParseGeometry(FeatureType::BEST_GEOMETRY);
  else
    CHECK( false, ("Feature", featureId, "not found"));

  return feature;
}

m2::PointD const & JointGraph::GetPoint(SegPoint const & vertex) const
{
  return GetFeature(vertex.GetFeatureId()).GetPoint(vertex.GetSegId());
}

} // namespace routing
