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
class FeaturePointsProviderImpl : public FeaturePointsProvider
{
public:
  explicit FeaturePointsProviderImpl(Index const & index);

  // implements FeaturePointsProvider
  m2::PointD const & GetPoint(uint32_t featureId, uint32_t segId) const override;
  size_t GetPointsCount(uint32_t featureId) const override;

private:
  FeatureType const & GetFeature(uint32_t featureId) const;
  FeatureType const & LoadFeature(uint32_t featureId) const;

  Index const & m_index;
  // @TODO |m_testMwmId| is added for writing prototype. It should be removed. MwmId from |m_mwmLocks|
  // should be used instead.
  MwmSet::MwmId const m_testMwmId;
  mutable map<uint32_t,FeatureType> m_features;
};

FeaturePointsProviderImpl::FeaturePointsProviderImpl(Index const & index)
  : m_index(index)
  , m_testMwmId(m_index.GetMwmIdByCountryFile(platform::CountryFile("Russia_Moscow")))
{}

size_t FeaturePointsProviderImpl::GetPointsCount(uint32_t featureId) const
{
  return GetFeature(featureId).GetPointsCount();
}

m2::PointD const & FeaturePointsProviderImpl::GetPoint(uint32_t featureId, uint32_t segId) const
{
  return GetFeature(featureId).GetPoint(segId);
}

FeatureType const & FeaturePointsProviderImpl::GetFeature(uint32_t featureId) const
{
  auto it = m_features.find(featureId);
  if ( it != m_features.end())
    return it->second;

  return LoadFeature(featureId);
}

FeatureType const & FeaturePointsProviderImpl::LoadFeature(uint32_t featureId) const
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

unique_ptr<FeaturePointsProvider> CreateFeaturePointsProvider(Index const & index)
{
  return make_unique<FeaturePointsProviderImpl>(index);
}

JointGraph::JointGraph(unique_ptr<FeaturePointsProvider> pointsProvider)
  : m_pointsProvider(move(pointsProvider))
{}

void JointGraph::GetOutgoingEdgesList(SegPoint const & vertexFrom, vector<SegEdge> & edges) const
{
  edges.clear();

  auto it = m_joints.find(vertexFrom.HashCode());
  if ( it != m_joints.end())
  {
    Joint const & joint = *it->second;
    for (size_t i = 0; i < joint.GetSize(); ++i)
    {
      AddAdjacentVertexes(joint.GetPoint(i),edges);
    }
  }
  else
  {
    AddAdjacentVertexes(vertexFrom,edges);
  }

//  LOG(LINFO, ("from => : ", vertexFrom, GetPoint(vertexFrom) ));
//  for ( SegEdge const & edge: edges)
//  {
//    LOG(LINFO, ("  to => : ", edge.GetTarget(), edge.GetWeight(), GetPoint(edge.GetTarget()) ));
//  }
}

void JointGraph::GetIngoingEdgesList(SegPoint const & vertex, vector<SegEdge> & edges) const
{
  GetOutgoingEdgesList(vertex,edges);
}

double JointGraph::HeuristicCostEstimate(SegPoint const & vertexFrom, SegPoint const & vertexTo) const
{
  return CalcDistance(GetPoint(vertexFrom), GetPoint(vertexTo));
}

vector<Junction> JointGraph::ConvertToGeometry(vector<SegPoint> const & vertexes) const
{
  vector<Junction> junctions;
  junctions.reserve(vertexes.size());
  for( SegPoint const & vertex: vertexes)
  {
    junctions.push_back(Junction(GetPoint(vertex),feature::kDefaultAltitudeMeters));
  }
  return junctions;
}

void JointGraph::AddAdjacentVertexes(SegPoint const & vertex, vector<SegEdge> & edges) const
{
  if (vertex.GetSegId() > 0)
  {
    SegPoint const nearbyVertex = ResolveVertex(vertex.GetFeatureId(), vertex.GetSegId() - 1);
    double const distance = HeuristicCostEstimate(vertex, nearbyVertex);
    edges.push_back(SegEdge(nearbyVertex, distance));
  }

  if (vertex.GetSegId() < m_pointsProvider->GetPointsCount(vertex.GetFeatureId()) - 1)
  {
    SegPoint const nearbyVertex = ResolveVertex(vertex.GetFeatureId(), vertex.GetSegId() + 1);
    double const distance = HeuristicCostEstimate(vertex, nearbyVertex);
    edges.push_back(SegEdge(nearbyVertex, distance));
  }
}

SegPoint JointGraph::ResolveVertex(SegPoint const & vertex) const
{
  return ResolveVertex(vertex.GetFeatureId(), vertex.GetSegId());
}

SegPoint JointGraph::ResolveVertex(uint32_t featureId, uint32_t segId) const
{
  SegPoint const unresolved(featureId,segId);
  auto it = m_joints.find(unresolved.HashCode());
  if ( it == m_joints.end())
    return unresolved;

  return it->second->GetPoint(0);
}

m2::PointD const & JointGraph::GetPoint(SegPoint const & vertex) const
{
  return m_pointsProvider->GetPoint(vertex.GetFeatureId(), vertex.GetSegId());
}

string DebugPrint(SegPoint const & vertex)
{
  ostringstream ss;
  ss << "V{" << vertex.GetFeatureId() << ", " << vertex.GetSegId() << "}";
  return ss.str();
}

} // namespace routing
