#pragma once


#include "quad_tree.h"
#include "grid_frame.h"
#include "geometry.h"
#include "grid_set.h"


/*
*
	class Foo {
	public:
		int gg;
		int Complexity() const { return 0.0; }

		void Hello();
	};
	class Bar {
		double gt;
	};

	class FieldType1 {
	public:
		double Complexity();

		FieldType1 SuperPose(const FieldType1& other) const;

		geom::YMonotonicGridSet2di GetAffectedGrids(const geom::GridFrame2d& grid_frame, const geom::BoxGridSet2di& roi) const;
	};

	LimitedField<Bar> a;
*/


//virtual geom::YMonotonicGridSet2di GetAffectedGrids(FieldId fid, const geom::GridFrame2d& grid_frame, const geom::BoxGridSet2di& roi) const = 0;

/*
class FieldType {
public:
	double Complexity();

	// field superposed with another one.
	void SuperPose(const FieldType& other);

	// A field can be simplified when the domain is restricted.
	void Simplify(const geom::AABox2d& box);

	// Being true means the field is empty.
	bool IsVoid() const;

	geom::YMonotonicGridSet2di GetAffectedGrids(const geom::GridFrame2d& grid_frame, const geom::BoxGridSet2di& roi) const;

	// RadiusOfBoundingbox? now it looks like a reference for gridframe unit.
	geom::AABox2d GetBoundingBox() const;
};
*/

template <class Type>
struct FieldTypeBase {
	template <class, class = void>
	struct HasFuncComplexity : std::false_type {};

	template <class T>
	struct HasFuncComplexity<T, std::void_t<decltype(std::declval<T>().Complexity())>> : std::true_type {};

	template <class, class = void>
	struct HasFuncSuperPose : std::false_type {};

	template <class T>
	struct HasFuncSuperPose<T, std::void_t<decltype(std::declval<T>().SuperPose())>> : std::true_type {};

	template <class, class = void>
	struct HasFuncSimplify : std::false_type {};

	template <class T>
	struct HasFuncSimplify<T, std::void_t<decltype(std::declval<T>().Simplify())>> : std::true_type {};

	template <class, class = void>
	struct HasFuncGetAffectedGrids : std::false_type {};

	template <class T>
	struct HasFuncGetAffectedGrids<T, std::void_t<decltype(std::declval<T>().GetAffectedGrids())>> : std::true_type {};

	template <class, class = void>
	struct HasFuncGetBoundingBox : std::false_type {};

	template <class T>
	struct HasFuncGetBoundingBox<T, std::void_t<decltype(std::declval<T>().GetBoundingBox())>> : std::true_type {};

	static constexpr bool kHasFuncComplexity = HasFuncComplexity<Type>::value;
	static constexpr bool kHasFuncSuperPose = HasFuncSuperPose<Type>::value;
	static constexpr bool kHasFuncGetAffectedGrids = HasFuncGetAffectedGrids<Type>::value;
	static constexpr bool kHasFuncSimplify = HasFuncSimplify<Type>::value;
	static constexpr bool kHasFuncGetBoundingBox = HasFuncGetBoundingBox<Type>::value;
};


// 我懂了， 这个事情， 有个选项叫 enforce single layer。
//
// 每个场图， general 的 定义是各个 层级的场的作用的叠加。
// 场的基本操作名为"细分". 某个层级的场片可以在细分时：
// 1, 本身复杂度的降低.
// 2, 细分后可以与子层级合并，通过预处理场减少复杂度.

// 细分不改变场的作用结果
// 单层场: 每个 x,y 位置只有一层场.
// 细分单层场 得到的仍然是 单层场.
// 细分多层场有可能会得到 单层场.

// 同理， 我们可以定义 2层 场: 顶多有两层的场。
// 细分只会减少层数


// 所以由这几个要素: 场， 细分序， 层数


 template <class FieldType>
class QuadTreeFieldMap {
public:
	static_assert(FieldTypeBase<FieldType>::kHasFuncComplexity, "FieldType missing "
		"double Complexity();");
	static_assert(FieldTypeBase<FieldType>::kHasFuncSuperPose, "FieldType missing "
		"void SuperPose(const FieldType& other);");
	static_assert(FieldTypeBase<FieldType>::kHasFuncSimplify, "FieldType missing "
		"void Simplify(const geom::Vec2d& box_center, double width);");
	static_assert(FieldTypeBase<FieldType>::kHasFuncGetAffectedGrids, "FieldType missing "
		"double GetAffectedGrids();");
	static_assert(FieldTypeBase<FieldType>::kHasFuncGetBoundingBox, "FieldType missing "
		"double GetBoundingBoxs();");


	static constexpr int kMaxDepth = 20;

	struct ConfigInternal {
		double origin_x = 0.0;
		double origin_y = 0.0;
		double root_box_width = 0.0;
		int depth = 0;
	};

	//TODO(some outer agent use this Config)
	struct Config {
		double x_min = 0.0;
		double x_max = 0.0;
		double y_min = 0.0;
		double y_max = 0.0;

		double origin_base = 0.0;
		double resolution = 0.0;

		bool IsValid() const {
			return x_max > x_min
				&& y_max > y_min
				&& resolution > 0.0
				&& origin_base > 0.0;
		}

		ConfigInternal ComputeConfigInternal() const {
			CHECK(IsValid());
			const double x0 = x_min - std::fmod(x_min, origin_base);
			const double y0 = y_min - std::fmod(y_min, origin_base);

			const double num_x_unit = std::ceil((x_max - x0) / resolution);
			const double num_y_unit = std::ceil((y_max - y0) / resolution);
			CHECK_GT(num_x_unit, 0.0);
			CHECK_GT(num_y_unit, 0.0);
			const int depth = static_cast<int>(std::ceil(std::log(std::max(num_x_unit, num_y_unit)))) + 1;
			CHECK_LT(depth, QuadTree<FieldType>::kMaxDepth) << "Resolution is too small";

			double root_box_width = resolution * (1 << (depth - 1));
			return ConfigInternal{
				x0,y0,root_box_width,depth
			};
		}

	};



	QuadTreeFieldMap(Config config, const std::vector<FieldType>& fields) :
		config_(std::move(config)),
		config_internal_(config_.ComputeConfigInternal()),
		map_(config_internal_.depth),
		unit_box_width_(map_.RootBoxWidthToUnitBoxWidth(config_internal_.root_box_width)) {
		CHECK_LE(config_internal_.depth, kMaxDepth);
	}

	// nullptr indicates no field present.

	struct FieldsAtPos {
		int num_fields = 0;
		std::array<const FieldType*, kMaxDepth> fields;
	};

	void GetFieldsAtPos(const geom::Vec2d& position) {
		DCHECK(fields != nullptr);
		FieldsAtPos result;

		QuadTree<FieldType>::ConstElementReferenceArray elements_at_pos =
			map_.GetElementsByPosition(
				position.x - config_internal_.origin_x,
				position.y - config_internal_.origin_y, 
				config_internal_.root_box_width);

		DCHECK_LE(elements_at_pos.num_elements, kMaxDepth);
		for (int i = 0; i < elements_at_pos.num_elements; ++i) {
			const FieldType* field = elements_at_pos.elements[i];
			if (!field->IsVoid()) {
				result.fields[result.num_fields] = field;
				result.num_fields += 1;
			}
		}
		return result;
	}

	// Add a field to the map.
	// return true when field is successfully added.
	// return false when container overflow.
	bool AddField(int level, int xid, int yid, FieldType field) {
		if (!map_.IsInside(level, xid, yid)) {
			return true;
		}
		if (field.IsVoid()) {
			return true;
		}

		FieldType* p_existing_field = map_.GetMutableElement(level, xid, yid);
		if (p_existing_field == nullptr) {
			return map_.AddElement(level, xid, yid, std::move(field)); 
		}
		else {
			p_existing_field->SuperPose(field);
			if (p_existing_field->IsVoid()) {
				map_.RemoveElement(level, xid, yid);
			}
			return true;
		}
	}

	// return true if field is successfully divided.
	// return false when there is too little space for further division. 
	bool DivideFieldAt(int level, int xid, int yid) {
		if (!map_.IsInside(level, xid, yid)) {
			return true;
		}

		if (level + 1 >= map_.depth()) {
			return true;
		}

		FieldType* p_field = map_.GetMutableElement(level, xid, yid);
		if (p_field == nullptr) {
			return true;
		}

		if (p_field->IsVoid()) {
			map_.RemoveElement(level, xid, yid);
			return true;
		}

		if (map_.max_num_elements() - map_.size() < 4) {
			// Too little space for further division.
			return false;
		}

		const int next_level = level + 1;
		const int nxid = xid << 1;
		const int nyid = yid << 1;

		const std::array<std::pair<int, int>, 4> children = { {0,0},{0,1},{1,0},{1,1} };

		for (const auto& child : children) {
			const int cxid = child.first + nxid;
			const int cyid = child.second + nyid;
			FieldType child_field = *p_field;
			child_field.Simplify(map_.BoxAt(next_level, cxid, cyid, unit_box_width_));

			if (!child_field.IsVoid()) {
				CHECK(AddField(next_level, cxid, cyid, std::move(child_field)));
			}
		}

		map_.RemoveElement(level, xid, yid);
	}


private:

	Config config_;
	ConfigInternal config_internal_;
	QuadTree<FieldType> map_;
	double unit_box_width_ = 0.0;
};
