#pragma once

#include "routing/num_mwm_id.hpp"

#include "geometry/rect2d.hpp"

#include "base/assert.hpp"
#include "base/checked_cast.hpp"

#include <string>
#include <vector>

#include "base/logging.hpp"
#include "geometry/mercator.hpp"

namespace routing
{
class MwmNeighbors final
{
public:
  template<typename GetNearbyMwmNames>
  MwmNeighbors(NumMwmIds const & numMwmIds, GetNearbyMwmNames && getNearbyMwmNames)
  {
    m_idToNeighbors.resize(numMwmIds.GetSize());

    numMwmIds.ForEachId([&](NumMwmId id){
      std::string const & name = numMwmIds.GetFile(id).GetName();
      vector<std::string> const nearbyNames = getNearbyMwmNames(name);

      auto & nearbyIds = m_idToNeighbors[GetIndex(id)];
      nearbyIds.reserve(nearbyNames.size());

      for (std::string const & nearbyName:nearbyNames)
      {
        platform::CountryFile const file(nearbyName);
        if (numMwmIds.ContainsFile(file))
          nearbyIds.push_back(numMwmIds.GetId(file));
      }

      if (id == 749)
      {
        int brk = 0;
      }
    });
  }

  std::vector<NumMwmId> const & GetNeighbors(NumMwmId numMwmId) const
  {
    return m_idToNeighbors[GetIndex(numMwmId)];
  }

private:
  size_t GetIndex(NumMwmId numMwmId) const
  {
    auto const index = base::asserted_cast<size_t>(numMwmId);
    ASSERT_LESS(index, m_idToNeighbors.size(), ());
    return index;
  }

  std::vector<std::vector<NumMwmId>> m_idToNeighbors;
};
}  // namespace routing
