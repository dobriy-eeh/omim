#include "astar_router.hpp"

#include "routing/base/astar_algorithm.hpp"
#include "routing/route.hpp"
#include "routing/turns_generator.hpp"
#include "routing/features_road_graph.hpp"
#include "routing/bicycle_directions.hpp"
#include "routing/bicycle_model.hpp"
#include "routing/pedestrian_directions.hpp"
#include "routing/pedestrian_model.hpp"
#include "routing/joint_graph.hpp"

#include "indexer/feature_altitude.hpp"

namespace
{
size_t constexpr kMaxRoadCandidates = 6;
} // namespace

namespace routing
{
AStarRouter::AStarRouter(Index const & index)
  : m_index(index)
  , m_roadGraph(make_unique<FeaturesRoadGraph>(index, IRoadGraph::Mode::ObeyOnewayTag, make_unique<PedestrianModelFactory>()))
  , m_directionsEngine(new PedestrianDirectionsEngine())
{
}

string AStarRouter::GetName() const
{
  return "AStar";
}

IRouter::ResultCode AStarRouter::CalculateRoute(m2::PointD const & startPoint,
                                                    m2::PointD const & /* startDirection */,
                                                    m2::PointD const & finalPoint,
                                                    RouterDelegate const & delegate, Route & route)
{
  vector<pair<Edge, Junction>> startVicinity;
  m_roadGraph->FindClosestEdges(startPoint,kMaxRoadCandidates, startVicinity);
  if (startVicinity.empty())
    return IRouter::StartPointNotFound;

  Edge const & startEdge = startVicinity[0].first;
  SegPoint const startVertex(startEdge.GetFeatureId().m_index, startEdge.GetSegId());

  vector<pair<Edge, Junction>> finalVicinity;
  m_roadGraph->FindClosestEdges(finalPoint,kMaxRoadCandidates, finalVicinity);
  if (finalVicinity.empty())
    return IRouter::EndPointNotFound;

  Edge const & finishEdge = finalVicinity[0].first;
  SegPoint const finishVertex(finishEdge.GetFeatureId().m_index, finishEdge.GetSegId());

  Junction const startPos = RoundJunction(startVicinity[0].first.GetStartJunction());
//  Junction const finalPos = RoundJunction(finalVicinity[0].first.GetStartJunction());
  Junction const finalPos = RoundJunction(startVicinity[0].first.GetEndJunction());

  vector<Junction> path;
//  path.push_back(Junction(startPoint, feature::kDefaultAltitudeMeters));
//  path.push_back(Junction(finalPoint, feature::kDefaultAltitudeMeters));
  path.push_back(startPos);
  path.push_back(finalPos);

  vector<Edge> routeEdges;
  routeEdges.push_back(startVicinity[0].first);

  function<void(SegPoint const &, SegPoint const &)> onVisitJunctionFn =
      [](SegPoint const & from, SegPoint const & to)
  {
  };

  JointGraph graph(m_index);
  AStarAlgorithm<JointGraph> algorithm;

  RoutingResult<SegPoint> routingResult;
  AStarAlgorithm<JointGraph>::Result const resultCode = algorithm.FindPathBidirectional(
      graph, startVertex, finishVertex, routingResult, delegate, onVisitJunctionFn);

  switch (resultCode) {
  case AStarAlgorithm<JointGraph>::Result::NoPath:
    return IRouter::RouteNotFound;
  case AStarAlgorithm<JointGraph>::Result::Cancelled:
    return IRouter::Cancelled;
  case AStarAlgorithm<JointGraph>::Result::OK:
    ReconstructRoute(move(path), route, delegate, routeEdges);
    return IRouter::NoError;
  }
}

void AStarRouter::ReconstructRoute(vector<Junction> && path, Route & route,
                                       my::Cancellable const & cancellable, vector<Edge> const & routeEdges) const
{
  CHECK(!path.empty(), ("Can't reconstruct route from an empty list of positions."));

  // By some reason there're two adjacent positions on a road with
  // the same end-points. This could happen, for example, when
  // direction on a road was changed.  But it doesn't matter since
  // this code reconstructs only geometry of a route.
  path.erase(unique(path.begin(), path.end()), path.end());
  if (path.size() == 1)
    path.emplace_back(path.back());

  Route::TTimes times;
  Route::TTurns turnsDir;
  vector<Junction> junctions;
  // @TODO(bykoianko) streetNames is not filled in Generate(). It should be done.
  Route::TStreets streetNames;

  m_directionsEngine->Generate(*m_roadGraph, path, times, turnsDir, junctions, cancellable, routeEdges);

  vector<m2::PointD> routeGeometry;
  JunctionsToPoints(junctions, routeGeometry);
  feature::TAltitudes altitudes;
  JunctionsToAltitudes(junctions, altitudes);

  route.SetGeometry(routeGeometry.begin(), routeGeometry.end());
  route.SetSectionTimes(move(times));
  route.SetTurnInstructions(move(turnsDir));
  route.SetStreetNames(move(streetNames));
  route.SetAltitudes(move(altitudes));
}
}  // namespace routing

