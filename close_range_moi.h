#pragma once


#include "quad_tree.h"
#include "grid_frame.h"
#include "geometry.h"
#include "grid_set.h"

// MOI: multi-object interaction
// There are many objects, each will interact with all others.
// Time complexity is too high.
// When only those "spacially close ranged" can interact with the target object. 
// It is possible to avoid enumerating them all.

// An object affects another one by first of all claim it impact in certain spacial domain,
// known as "field", the other object will then get affected by the field.

// Multiple object's field will don't have to be necessarily adding up.
// But there is always a way to tell their joint impact to the space.
// To tell the final field. etc.


// 1, Field exists in space, for limited space, field can be expressed/calculated in simpler way
// 2, Each field has limited spacial impact.


namespace close_range_moi {
	// 这个类最好是 场 到 场
	class ObjectFieldContainer {
	public:
		class Field {
			virtual bool CanNotAffect(const geom::Vec2d& box_center, double box_width) = 0; // redundant.
			virtual geom::YMonotonicGridSet2di GetAffectedGrids(const geom::GridFrame2d& grid_frame, const geom::BoxGridSet2di& roi) = 0;
			virtual double Complexity() const;
		};

		virtual geom::YMonotonicGridSet2di GetAffectedGrids(const Field& field, 
			const geom::GridFrame2d& grid_frame, const geom::BoxGridSet2di& roi) = 0;

		// How a field can be simplified
		virtual Field CalculateSuperPosedField(const std::vector<Field>& fields, const geom::Vec2d& box_center, double box_width) = 0;
	};

	class LimitedSpaceField {
		virtual geom::YMonotonicGridSet2di GetAffectedGrids(const geom::GridFrame2d& grid_frame, const geom::BoxGridSet2di& roi) const = 0;
		virtual double Complexity() const = 0;
		virtual

		virtual FromFieldSuperPosition(const std::vector<LimitedSpaceField*>& fields, const geom::Vec2d& box_center, double box_width) = 0;
	};

	// 场的定义和 容器拧在一起了。 因为容器实现取决于定义。
	// 定义是需要被派生的， 这里的最基本的方法， affected grid以外， 有一个要求两两叠加的功能。
	// 叠加功能是基本方法，所以基类至少在派生的时候是同时确定所有场的类型的。

	// 由于每一种场的类型不一样， 基类的接口类型只能是抽象的， 返回类型也如此。

	// 叠加这件事， 需要知道每个场的细节实现， 没法用抽象的接口来做， 所以在派生的 时候， 
	// 派生类需要清楚的知道接口类型是什么，  所以这个接口的类型要在派生时同步确定。
	// 这是C++ 做不了的， 所以这个 “派生时同时确定接口” 的能力是没有的。

	// 接口只能是确定类型. 即泛用指针。 指针的使用也要由派生行为来确定。

	// 这也就是 容器和定义 耦合的原因。
	class Field {
		virtual geom::YMonotonicGridSet2di GetAffectedGrids(const geom::GridFrame2d& grid_frame, const geom::BoxGridSet2di& roi) const = 0;

	};


	class LimitedSpaceFieldContainer {

	};

	using FieldId = int;
	class LimitedSpaceFieldManager {
		virtual geom::YMonotonicGridSet2di GetAffectedGrids(FieldId fid, const geom::GridFrame2d& grid_frame, const geom::BoxGridSet2di& roi) const = 0;
		virtual double Complexity(FieldId fid) const = 0;

		// Always assume the superposition is simpler than a vector of fields.
		virtual int FieldSuperPosition(const std::vector<FieldId>& fids, const geom::Vec2d& box_center, double box_width) = 0;
	};
	
	class LimitedSpaceFieldMap {
		// Only handle index
		// input: vector of FieldId as initial fields
		// output: me,  can accept querry in vec2d, lead you to a field very fast.
	};



	// The quad tree will 


	class QuadTreeMOI2d {
	public:



		class Object {
			virtual bool CanNotAffect(const geom::Vec2d& box_center, double box_width) = 0;
			virtual geom::YMonotonicGridSet2di GetAffectedGrids(const geom::GridFrame2d& grid_frame, const geom::BoxGridSet2di& roi) = 0;
		};
		




	};


}  // namespace close_range_space


