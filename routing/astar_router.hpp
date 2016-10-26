#pragma once

#include "indexer/index.hpp"

#include "routing/router.hpp"
#include "routing/road_graph.hpp"
#include "routing/directions_engine.hpp"
#include "routing/pedestrian_directions.hpp"

#include "std/unique_ptr.hpp"
#include "std/vector.hpp"

namespace routing
{
class AStarRouter : public IRouter
{
public:
  AStarRouter(Index const & index);

  // IRouter overrides:
  string GetName() const override;
  ResultCode CalculateRoute(m2::PointD const & startPoint, m2::PointD const & startDirection,
                            m2::PointD const & finalPoint, RouterDelegate const & delegate,
                            Route & route) override;

private:
  void ReconstructRoute(vector<Junction> && junctions, Route & route,
                        my::Cancellable const & cancellable, vector<Edge> const & routeEdges) const;

  unique_ptr<IRoadGraph> const m_roadGraph;
  unique_ptr<PedestrianDirectionsEngine> const m_directionsEngine;
};
}  // namespace routing

