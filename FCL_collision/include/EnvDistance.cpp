#include "EnvDistance.h"
#include <fcl/geometry/shape/cylinder.h>
#include <string>
#include <sys/types.h>
#include <stdio.h>
#include <errno.h>
#include <cmath>
#include "DistanceHelper.h"

namespace Clash {
	void EnvDistance::update(std::vector<Clash::CylinderModel>& cms)
	{
		if (env.size()) {
			int len = env.size();
			for (int i = 0; i < len; i++) {
				delete env[i];
			}
			env.clear();
		}

		for (const auto& cm : cms) {
			std::shared_ptr<fcl::CollisionGeometry<double>> g = std::make_shared<fcl::Cylinder<double>>(cm.radius, cm.height);
			fcl::CollisionObject<double>* c = new fcl::CollisionObject<double>(g);
			c->setRotation(cm.rotation);
			c->setTranslation(cm.mv);
			// std::cout << "env cylinder mv => " << cm.mv << std::endl;
			env.push_back(c);
		}
	}

	std::tuple<double, std::vector<double>> EnvDistance::calcDistance(std::vector<fcl::CollisionObject<double>*>& chains)
	{
		return Clash::DistanceHelper::calcDistance(env, chains);
	}
}