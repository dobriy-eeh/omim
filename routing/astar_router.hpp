#pragma once

#include "routing/directions_engine.hpp"
#include "routing/road_graph.hpp"
#include "routing/router.hpp"
#include "routing/vehicle_model.hpp"

#include "indexer/index.hpp"

#include "std/shared_ptr.hpp"
#include "std/unique_ptr.hpp"
#include "std/vector.hpp"

namespace routing
{
class IndexGraph;

class AStarRouter : public IRouter
{
public:
  AStarRouter(Index const & index, TCountryFileFn const & countryFileFn,
              shared_ptr<VehicleModelFactory> vehicleModelFactory,
              unique_ptr<IDirectionsEngine> directionsEngine);

  // IRouter overrides:
  string GetName() const override;
  ResultCode CalculateRoute(m2::PointD const & startPoint, m2::PointD const & startDirection,
                            m2::PointD const & finalPoint, RouterDelegate const & delegate,
                            Route & route) override;

private:
  bool FindClosestEdge(Edge & closestEdge, m2::PointD const & point) const;
  bool LoadIndex(IndexGraph & graph, MwmSet::MwmId const & mwmId);

  Index const & m_index;
  TCountryFileFn m_countryFileFn;
  shared_ptr<IRoadGraph> m_roadGraph;
  shared_ptr<VehicleModelFactory> m_vehicleModelFactory;
  unique_ptr<IDirectionsEngine> m_directionsEngine;
};

unique_ptr<IRouter> CreateCarAStarBidirectionalRouter(Index & index,
                                                      TCountryFileFn const & countryFileFn);
}  // namespace routing
