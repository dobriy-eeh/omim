#include "testing/testing.hpp"

#include "routing/base/astar_algorithm.hpp"
#include "routing/joint_graph.hpp"

#include "base/assert.hpp"

namespace routing_test
{
using namespace routing;

namespace
{
class TestFeaturePointsProvider : public FeaturePointsProvider
{
public:
  // implements FeaturePointsProvider
  m2::PointD const & GetPoint(uint32_t featureId, uint32_t segId) const override;
  size_t GetPointsCount(uint32_t featureId) const override;

  void AddRoad(uint32_t featureId, vector<m2::PointD> const & points);

private:
  map<uint32_t,vector<m2::PointD>> m_features;
};

size_t TestFeaturePointsProvider::GetPointsCount(uint32_t featureId) const
{
  auto it = m_features.find(featureId);
  if ( it == m_features.end())
  {
    ASSERT( false, ("Can't find feature", featureId));
    return 0;
  }

  return it->second.size();
}

m2::PointD const & TestFeaturePointsProvider::GetPoint(uint32_t featureId, uint32_t segId) const
{
  auto it = m_features.find(featureId);
  if ( it == m_features.end())
  {
    ASSERT( false, ("Can't find feature", featureId));
    static m2::PointD invalidResult(-1.0,-1.0);
    return invalidResult;
  }

  ASSERT( segId < it->second.size(), ("Too large segId =", segId, ", size =", it->second.size(), ", featureId =", featureId));
  return it->second[segId];
}

void TestFeaturePointsProvider::AddRoad(uint32_t featureId, vector<m2::PointD> const & points)
{
  auto it = m_features.find(featureId);
  if ( it != m_features.end())
  {
    ASSERT( false, ("Already contains feature", featureId));
    return;
  }

  m_features[featureId] = points;
}

shared_ptr<Joint> MakeJoint(vector<SegPoint> const & points)
{
  shared_ptr<Joint> joint = make_shared<Joint>();
  for (auto const & point: points)
  {
    joint->AddPoint(point);
  }
  return joint;
}

void CheckRoute(JointGraph const & graph, SegPoint const & start, SegPoint const & finish, size_t expectedLength)
{
  LOG(LINFO, ("Check route", start, "=>", finish));

  AStarAlgorithm<JointGraph> algorithm;
  RoutingResult<SegPoint> routingResult;

  auto onVisitFn = [](SegPoint const & from, SegPoint const & to)
  {
  };

  AStarAlgorithm<JointGraph>::Result const resultCode = algorithm.FindPath(graph, graph.ResolveVertex(start), graph.ResolveVertex(finish), routingResult, {}, onVisitFn);

  TEST_EQUAL(resultCode, AStarAlgorithm<JointGraph>::Result::OK, ());
  TEST_EQUAL(routingResult.path.size(), expectedLength, ());
}

void CheckRouteBothWays(JointGraph const & graph, SegPoint const & start, SegPoint const & finish, size_t expectedLength)
{
  CheckRoute(graph,start,finish,expectedLength);
  CheckRoute(graph,finish,start,expectedLength);
}

void CheckRouteAllCombos(JointGraph const & graph, vector<SegPoint> const & ends, size_t expectedLength)
{
  for ( auto const & start: ends)
  {
    for ( auto const & finish: ends)
    {
      CheckRouteBothWays(graph,start,finish, start == finish ? 1 : expectedLength);
    }
  }
}

}  // namespace


//            R1:
//
//            -2
//            -1
//  R0: -2 -1  0  1  2
//             1
//             2
//
UNIT_TEST(FindPathCross)
{
  unique_ptr<TestFeaturePointsProvider> provider = make_unique<TestFeaturePointsProvider>();
  provider->AddRoad(0, {{-2.0, 0.0}, {-1.0, 0.0}, {0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}});
  provider->AddRoad(1, {{0.0, -2.0}, {-1.0, 0.0}, {0.0, 0.0}, {0.0, 1.0}, {0.0, 2.0}});

  JointGraph graph(move(provider));

  graph.AddJoint(MakeJoint({{0,2},{1,2}}));

  CheckRouteAllCombos(graph,{{0,0}, {0,4}, {1,0}, {1,4}}, 5);
  CheckRouteAllCombos(graph,{{0,1}, {0,3}, {1,1}, {1,3}}, 3);
  CheckRouteBothWays(graph,{0,0}, {1,1}, 4);
}

//      R4  R5  R6  R7
//
//  R0:  0 - * - * - *
//       |   |   |   |
//  R1:  * - 1 - * - *
//       |   |   |   |
//  R2   * - * - 2 - *
//       |   |   |   |
//  R3   * - * - * - 3
//
UNIT_TEST(FindPathManhattan)
{
  uint32_t constexpr kCitySize = 4;
  unique_ptr<TestFeaturePointsProvider> provider = make_unique<TestFeaturePointsProvider>();
  for ( uint32_t i = 0; i < kCitySize; ++i )
  {
    vector<m2::PointD> horizontalRoad;
    vector<m2::PointD> verticalRoad;
    for ( uint32_t j = 0; j < kCitySize; ++j )
    {
      horizontalRoad.push_back({(double)i, (double)j});
      verticalRoad.push_back({(double)j, (double)i});
    }
    provider->AddRoad(i, horizontalRoad);
    provider->AddRoad(i+kCitySize, verticalRoad);
  }

  JointGraph graph(move(provider));

  for ( uint32_t i = 0; i < kCitySize; ++i )
  {
    for ( uint32_t j = 0; j < kCitySize; ++j )
    {
      graph.AddJoint(MakeJoint({{i,j},{j+kCitySize,i}}));
    }
  }

  CheckRoute(graph,{4,0}, {4,1}, 2);

  for ( uint32_t i = 0; i < kCitySize * 2; ++i )
  {
    for ( uint32_t start = 0; start < kCitySize - 1; ++start )
    {
      for ( uint32_t finish = start + 1; finish < kCitySize; ++finish )
      {
        CheckRouteBothWays(graph,{i,start}, {i,finish}, finish-start+1);
      }
    }
  }

  for ( uint32_t i = 0; i < kCitySize-1; ++i )
  {
    CheckRouteBothWays(graph,{i,i}, {i+1,i+1}, 3);
    CheckRouteBothWays(graph,{i,kCitySize-1-i}, {i+1,kCitySize-2-i}, 3);
  }

  CheckRouteBothWays(graph,{0,0}, {kCitySize-1,kCitySize-1}, kCitySize*2-1);
  CheckRouteBothWays(graph,{kCitySize-1,0}, {0,kCitySize-1}, kCitySize*2-1);

  CheckRouteBothWays(graph,{0,1}, {2,2}, 4);
}

}  // namespace routing_test
