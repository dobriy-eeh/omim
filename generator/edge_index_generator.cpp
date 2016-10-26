#include "generator/edge_index_generator.hpp"

#include "routing/features_road_graph.hpp"
#include "routing/joint_graph.hpp"
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
using namespace routing;

namespace
{
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
      m_joints[locationKey].AddPoint(SegPoint(id,fromSegId));
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
    for(auto it = m_joints.begin(); it != m_joints.end();)
    {
      if ( it->second.GetSize() < 2)
        it = m_joints.erase(it);
      else
        ++it;
    }

    map<size_t,vector<Joint>> jointsBySize;

    size_t jointsNumber = 0;
    for(auto it = m_joints.begin(); it != m_joints.end();++it)
    {
      size_t const jointSize = it->second.GetSize();
      if ( jointSize < 2)
        continue;

      vector<Joint> & joints = jointsBySize[jointSize];
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
        for ( size_t i = 0; i < joint.GetSize(); ++i)
        {
          SegPoint const & point = joint.GetPoint(i);
          if ( maxSegId < point.GetSegId() )
            maxSegId = point.GetSegId();
          if ( maxFeatureId < point.GetFeatureId() )
            maxFeatureId = point.GetFeatureId();
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
    WriteToSink(sink, static_cast<uint32_t>(m_joints.size()));
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
  map<uint64_t,Joint> m_joints;
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
