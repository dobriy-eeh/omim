#include "generator/edge_index_generator.hpp"

#include "routing/features_road_graph.hpp"
#include "routing/road_graph.hpp"
#include "routing/routing_helpers.hpp"

#include "indexer/feature.hpp"
#include "indexer/feature_edge_index.hpp"
#include "indexer/feature_processor.hpp"
#include "indexer/index.hpp"
#include "indexer/scales.hpp"

#include "geometry/rect2d.hpp"

#include "coding/file_name_utils.hpp"

#include "platform/country_file.hpp"
#include "platform/local_country_file.hpp"

#include "base/logging.hpp"
#include "base/stl_helpers.hpp"

#include "std/algorithm.hpp"
#include "std/unique_ptr.hpp"
#include "std/vector.hpp"

using namespace feature;
using namespace platform;

namespace
{
class SegPoint
{
public:
  uint32_t m_featureId;
  uint32_t m_segId;

  SegPoint() = default;
  SegPoint(const SegPoint &) = default;
  SegPoint(uint32_t featureId, uint32_t segId)
    : m_featureId(featureId)
    , m_segId(segId)
  {}

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

  uint64_t calcKey() const
  {
    return (static_cast<uint64_t>(m_featureId) << 32) + static_cast<uint64_t>(m_segId);
  }

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
};

class RouteJoint
{
public:
  vector<SegPoint> m_points;

  bool operator == (RouteJoint const & joint) const
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

  bool operator != (RouteJoint const & joint) const
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
};

uint64_t calcLocationKey(m2::PointD const & point)
{
  m2::PointI const pointI( routing::PointDToPointI(point));
  return (static_cast<uint64_t>(pointI.y) << 32) + static_cast<uint64_t>(pointI.x);
}

class Processor
{
public:
  Processor(string const & dir, string const & country)
    : m_roadGraph(m_index, routing::IRoadGraph::Mode::IgnoreOnewayTag,
                  make_unique<routing::PedestrianModelFactory>())
  {
    LocalCountryFile localCountryFile(dir, CountryFile(country), 1 /* version */);
    m_index.RegisterMap(localCountryFile);
    vector<shared_ptr<MwmInfo>> info;
    m_index.GetMwmsInfo(info);
    CHECK_EQUAL(info.size(), 1, ());
    CHECK(info[0], ());
    m_limitRect = info[0]->m_limitRect;
    LOG(LINFO, ("m_limitRect =", m_limitRect));
  }

  void operator()(FeatureType const & f)
  {
    if (!m_roadGraph.IsRoad(f))
      return;

    uint32_t const id = f.GetID().m_index;
    if (m_featuresCount != 0 && m_featuresCount % 1000 == 0)
      LOG(LINFO, ("id =", id, "road features", m_featuresCount));

    ++m_featuresCount;

    f.ParseGeometry(FeatureType::BEST_GEOMETRY);
    size_t const pointsCount = f.GetPointsCount();
    if (pointsCount == 0)
      return;

    for (size_t fromSegId = 0; fromSegId < pointsCount; ++fromSegId)
    {
      uint64_t const locationKey = calcLocationKey(f.GetPoint(fromSegId));
      SegPoint const segPoint(id,fromSegId);
      RouteJoint & joint = m_joints[locationKey];
      joint.m_points.push_back(segPoint);
    }
  }

  vector<FeatureOutgoingEdges> const & GetOutgoingEdges()
  {
    return m_outgoingEdges;
  }

  m2::RectD const & GetLimitRect()
  {
    return m_limitRect;
  }

  void ForEachFeature()
  {
    m_index.ForEachInScale(*this, m_roadGraph.GetStreetReadScale());
  }

  void PrintStatistics()
  {
    map<size_t,vector<RouteJoint>> jointsBySize;

    size_t jointsNumber = 0;
    for(auto it = m_joints.begin(); it != m_joints.end();++it)
    {
      size_t const jointSize = it->second.m_points.size();
      if ( jointSize < 2)
        continue;

      vector<RouteJoint> & joints = jointsBySize[jointSize];
      joints.push_back(it->second);
      ++jointsNumber;
    }
    LOG(LINFO, ("joints number =", jointsNumber));

    int maxSegId = 0;
    int maxFeatureId = 0;
    for(auto it = jointsBySize.begin(); it != jointsBySize.end();++it)
    {
      for ( auto const & joint: it->second )
      {
        for ( auto const & point: joint.m_points )
        {
          if ( maxSegId < point.m_segId )
            maxSegId = point.m_segId;
          if ( maxFeatureId < point.m_featureId )
            maxFeatureId = point.m_featureId;
        }
      }
      LOG(LINFO, ("joints",it->first,":", it->second.size()));
    }

    LOG(LINFO, ("maxSegId =", maxSegId));
    LOG(LINFO, ("maxFeatureId =", maxFeatureId));
  }

  template <class TSink>
  void SerializeJoints(TSink & sink) const
  {
    for(auto it = m_joints.begin(); it != m_joints.end();++it)
    {
      it->second.Serialize(sink);
    }
  }

private:
  Index m_index;
  routing::FeaturesRoadGraph m_roadGraph;
  vector<FeatureOutgoingEdges> m_outgoingEdges;
  m2::RectD m_limitRect;
  map<uint64_t,RouteJoint> m_joints;
  size_t m_featuresCount = 0;
};
}  // namespace

namespace routing
{
void BuildOutgoingEdgeIndex(string const & dir, string const & country)
{
  LOG(LINFO, ("dir =", dir, "country", country));
  try
  {
    Processor processor(dir, country);
    string const datFile = my::JoinFoldersToPath(dir, country + DATA_FILE_EXTENSION);
    LOG(LINFO, ("datFile =", datFile));
    processor.ForEachFeature();
    processor.PrintStatistics();

    FilesContainerW cont(datFile, FileWriter::OP_WRITE_EXISTING);
    FileWriter w = cont.GetWriter(EDGE_INDEX_FILE_TAG);

    EdgeIndexHeader header(0);
    header.Serialize(w);

    processor.SerializeJoints(w);
  }
  catch (RootException const & e)
  {
    LOG(LERROR, ("An exception happened while creating", EDGE_INDEX_FILE_TAG, "section:", e.what()));
  }
}
}  // namespace routing
