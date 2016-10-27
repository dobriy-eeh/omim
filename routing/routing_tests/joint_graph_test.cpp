#include "testing/testing.hpp"

#include "routing/base/astar_algorithm.hpp"
#include "routing/joint_graph.hpp"

#include "base/assert.hpp"

namespace routing_test
{
using namespace routing;

namespace
{
class FeaturePointsProviderTest : public FeaturePointsProvider
{
public:
  // implements FeaturePointsProvider
  m2::PointD const & GetPoint(uint32_t featureId, uint32_t segId) const override;
  size_t GetPointsCount(uint32_t featureId) const override;

  void AddFeature(uint32_t featureId, vector<m2::PointD> && points);

private:
  map<uint32_t,vector<m2::PointD>> m_features;
};

size_t FeaturePointsProviderTest::GetPointsCount(uint32_t featureId) const
{
  auto it = m_features.find(featureId);
  if ( it == m_features.end())
  {
    ASSERT( false, ("Can't find feature", featureId));
    return 0;
  }

  return it->second.size();
}

m2::PointD const & FeaturePointsProviderTest::GetPoint(uint32_t featureId, uint32_t segId) const
{
  auto it = m_features.find(featureId);
  if ( it == m_features.end())
  {
    ASSERT( false, ("Can't find feature", featureId));
    static m2::PointD invalidResult(-1.0,-1.0);
    return invalidResult;
  }

  return it->second[segId];
}

void FeaturePointsProviderTest::AddFeature(uint32_t featureId, vector<m2::PointD> && points)
{
  auto it = m_features.find(featureId);
  if ( it != m_features.end())
  {
    ASSERT( false, ("eaturePointsProvider already contains feature", featureId));
    return;
  }

  m_features[featureId] = points;
}

void CheckRoute(JointGraph const & graph, SegPoint const & start, SegPoint const & finish, size_t expectedLength)
{
  AStarAlgorithm<JointGraph> algorithm;
  RoutingResult<SegPoint> routingResult;

  auto onVisitFn = [](SegPoint const & from, SegPoint const & to)
  {
    LOG(LINFO, ("Visited", from, "=>", to));
  };

  AStarAlgorithm<JointGraph>::Result const resultCode = algorithm.FindPath(graph, start, finish, routingResult, {}, onVisitFn);

  TEST(resultCode == AStarAlgorithm<JointGraph>::Result::OK, (", start =", start,"finish=", finish));
  TEST_EQUAL(routingResult.path.size(), expectedLength, (", start =", start,"finish=", finish));
  LOG(LINFO, ("Route", start, "=>", finish, "ok"));
}
}  // namespace

UNIT_TEST(FindPathSimpleCross)
{
  LOG(LINFO, ("Hello world RouteSimpleCross"));

  unique_ptr<FeaturePointsProviderTest> provider = make_unique<FeaturePointsProviderTest>();
  provider->AddFeature(0, {{-2.0, 0.0}, {-1.0, 0.0}, {0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}});
  provider->AddFeature(1, {{0.0, -2.0}, {-1.0, 0.0}, {0.0, 0.0}, {0.0, 1.0}, {0.0, 2.0}});

  JointGraph graph(move(provider));

  shared_ptr<Joint> joint = make_shared<Joint>();
  joint->AddPoint({0,2});
  joint->AddPoint({1,2});
  graph.AddJoint(joint);

  CheckRoute(graph,{0,0}, {0,0}, 1);
  CheckRoute(graph,{1,3}, {1,3}, 1);
  CheckRoute(graph,{0,2}, {0,2}, 1);

  CheckRoute(graph,{0,0}, {0,4}, 5);
  CheckRoute(graph,{0,0}, {1,0}, 5);
  CheckRoute(graph,{0,0}, {1,4}, 5);
  CheckRoute(graph,{0,4}, {0,0}, 5);
  CheckRoute(graph,{1,4}, {0,4}, 5);
  CheckRoute(graph,{0,4}, {1,0}, 5);

  CheckRoute(graph,{0,1}, {0,3}, 3);
  CheckRoute(graph,{0,1}, {1,1}, 3);
  CheckRoute(graph,{0,1}, {1,3}, 3);
  CheckRoute(graph,{0,3}, {0,1}, 3);
  CheckRoute(graph,{1,3}, {0,3}, 3);
  CheckRoute(graph,{0,3}, {1,1}, 3);
}

}  // namespace routing_test
