#include "astar_router.hpp"

#include "routing/base/astar_algorithm.hpp"
#include "routing/base/astar_progress.hpp"
#include "routing/bicycle_directions.hpp"
#include "routing/bicycle_model.hpp"
#include "routing/car_model.hpp"
#include "routing/features_road_graph.hpp"
#include "routing/index_graph.hpp"
#include "routing/pedestrian_model.hpp"
#include "routing/route.hpp"
#include "routing/turns_generator.hpp"

#include "indexer/feature_altitude.hpp"
#include "indexer/routing_section.hpp"

#include "geometry/distance.hpp"
#include "geometry/point2d.hpp"

namespace
{
size_t constexpr kMaxRoadCandidates = 6;
float constexpr kProgressInterval = 2;

using namespace routing;

vector<Junction> ConvertToJunctions(IndexGraph const & graph, vector<JointId> const & joints)
{
  vector<FSegId> const & fsegs = graph.RedressRoute(joints);

  vector<Junction> junctions;
  junctions.reserve(fsegs.size());

  Geometry const & geometry = graph.GetGeometry();
  for (FSegId const & fseg : fsegs)
    junctions.emplace_back(geometry.GetPoint(fseg), feature::kDefaultAltitudeMeters);

  return junctions;
}
}  // namespace

namespace routing
{
AStarRouter::AStarRouter(Index const & index, TCountryFileFn const & countryFileFn,
                         shared_ptr<VehicleModelFactory> vehicleModelFactory,
                         unique_ptr<IDirectionsEngine> directionsEngine)
  : m_index(index)
  , m_countryFileFn(countryFileFn)
  , m_roadGraph(
        make_shared<FeaturesRoadGraph>(index, IRoadGraph::Mode::ObeyOnewayTag, vehicleModelFactory))
  , m_vehicleModelFactory(vehicleModelFactory)
  , m_directionsEngine(move(directionsEngine))
{
}

string AStarRouter::GetName() const { return "AStar"; }

IRouter::ResultCode AStarRouter::CalculateRoute(m2::PointD const & startPoint,
                                                m2::PointD const & /* startDirection */,
                                                m2::PointD const & finalPoint,
                                                RouterDelegate const & delegate, Route & route)
{
  Edge startEdge = Edge::MakeFake({},{});
  if (!FindClosestEdge(startEdge, startPoint))
    return IRouter::StartPointNotFound;

  Edge finishEdge = Edge::MakeFake({},{});
  if (!FindClosestEdge(finishEdge, finalPoint))
    return IRouter::EndPointNotFound;

  FSegId const start(startEdge.GetFeatureId().m_index, startEdge.GetSegId());
  FSegId const finish(finishEdge.GetFeatureId().m_index, finishEdge.GetSegId());

  string const country = m_countryFileFn(startPoint);
  MwmSet::MwmId const mwmId(m_index.GetMwmIdByCountryFile(platform::CountryFile(country)));
  IndexGraph graph(
      CreateGeometry(m_index, mwmId, m_vehicleModelFactory->GetVehicleModelForCountry(country)));

  if (!LoadIndex(graph, mwmId))
    return IRouter::RouteFileNotExist;

  AStarProgress progress(0, 100);

  auto onVisitJunctionFn = [&delegate, &progress, &graph](JointId const & from,
                                                          JointId const & /*finish*/) {
    m2::PointD const & point = graph.GetPoint(from);
    auto const lastValue = progress.GetLastValue();
    auto const newValue = progress.GetProgressForDirectedAlgo(point);
    if (newValue - lastValue > kProgressInterval)
    {
      delegate.OnProgress(newValue);
    }
    delegate.OnPointCheck(point);
  };

  AStarAlgorithm<IndexGraph> algorithm;

  RoutingResult<JointId> routingResult;
  AStarAlgorithm<IndexGraph>::Result const resultCode =
      algorithm.FindPathBidirectional(graph, graph.InsertJoint(start), graph.InsertJoint(finish),
                                      routingResult, delegate, onVisitJunctionFn);

  switch (resultCode)
  {
  case AStarAlgorithm<IndexGraph>::Result::NoPath: return IRouter::RouteNotFound;
  case AStarAlgorithm<IndexGraph>::Result::Cancelled: return IRouter::Cancelled;
  case AStarAlgorithm<IndexGraph>::Result::OK:
    vector<Junction> path = ConvertToJunctions(graph, routingResult.path);
    ReconstructRoute(m_directionsEngine.get(), *m_roadGraph, move(path), route, delegate);
    return IRouter::NoError;
  }
}

bool AStarRouter::FindClosestEdge(Edge & closestEdge, m2::PointD const & point) const
{
  vector<pair<Edge, Junction>> candidates;
  m_roadGraph->FindClosestEdges(point, kMaxRoadCandidates, candidates);

  double minDistance = numeric_limits<double>::max();
  size_t minIndex = candidates.size();

  for (size_t i = 0; i < candidates.size(); ++i)
  {
    Edge const & edge = candidates[i].first;
    m2::DistanceToLineSquare<m2::PointD> squareDistance;
    squareDistance.SetBounds(edge.GetStartJunction().GetPoint(), edge.GetEndJunction().GetPoint());
    double const distance = squareDistance(point);
    if (distance < minDistance)
    {
      minDistance = distance;
      minIndex = i;
    }
  }

  if (minIndex < candidates.size())
  {
    closestEdge = candidates[minIndex].first;
    return true;
  }

  return false;
}

bool AStarRouter::LoadIndex(IndexGraph & graph, MwmSet::MwmId const & mwmId)
{
  MwmSet::MwmHandle mwmHandle = m_index.GetMwmHandleById(mwmId);
  MwmValue const * mwmValue = mwmHandle.GetValue<MwmValue>();
  if (!mwmValue)
  {
    LOG(LERROR, ("mwmValue == null"));
    return false;
  }

  try
  {
    my::Timer timer;
    FilesContainerR::TReader reader(mwmValue->m_cont.GetReader(ROUTING_FILE_TAG));
    ReaderSource<FilesContainerR::TReader> src(reader);
    feature::RoutingSectionHeader header;
    header.Deserialize(src);
    graph.Deserialize(src);
    LOG(LINFO, ("Routing section loaded in ", timer.ElapsedSeconds(), "seconds"));
    return true;
  }
  catch (Reader::OpenException const & e)
  {
    LOG(LERROR, ("File", mwmValue->GetCountryFileName(), "Error while reading", ROUTING_FILE_TAG,
                 "section.", e.Msg()));
    return false;
  }
}

unique_ptr<IRouter> CreateCarAStarBidirectionalRouter(Index & index,
                                                      TCountryFileFn const & countryFileFn)
{
  shared_ptr<VehicleModelFactory> vehicleModelFactory = make_shared<CarModelFactory>();
  // @TODO Bicycle turn generation engine is used now. It's ok for the time being.
  // But later a special car turn generation engine should be implemented.
  unique_ptr<IDirectionsEngine> directionsEngine = make_unique<BicycleDirectionsEngine>(index);
  unique_ptr<IRouter> router = make_unique<AStarRouter>(
      index, countryFileFn, move(vehicleModelFactory), move(directionsEngine));
  return router;
}
}  // namespace routing
